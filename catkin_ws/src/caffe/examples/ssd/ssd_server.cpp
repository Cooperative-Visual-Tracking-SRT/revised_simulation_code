// This is a demo code for using a SSD model to do detection.
// The code is modified from examples/cpp_classification/classification.cpp.
// Usage:
//    ssd_detect [FLAGS] model_file weights_file list_file
//
// where model_file is the .prototxt file defining the network architecture, and
// weights_file is the .caffemodel file containing the network parameters, and
// list_file contains a list of image files with the format as follows:
//    folder/img1.JPEG
//    folder/img2.JPEG
// list_file can also contain a list of video files with the format as follows:
//    folder/video1.mp4
//    folder/video2.mp4
//
#include <caffe/caffe.hpp>
#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif  // USE_OPENCV
#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#ifdef USE_OPENCV
using namespace caffe;  // NOLINT(build/namespaces)



class Detector {
 public:
  Detector(const string& model_file,
           const string& weights_file,
           const string& mean_file,
           const string& mean_value);

  std::vector<vector<float> > Detect(const cv::Mat& img);

 private:
  void SetMean(const string& mean_file, const string& mean_value);

  void WrapInputLayer(std::vector<cv::Mat>* input_channels);

  void Preprocess(const cv::Mat& img,
                  std::vector<cv::Mat>* input_channels);

 private:
  shared_ptr<Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
};

Detector::Detector(const string& model_file,
                   const string& weights_file,
                   const string& mean_file,
                   const string& mean_value) {
#ifdef CPU_ONLY
  Caffe::set_mode(Caffe::CPU);
#else
  Caffe::set_mode(Caffe::GPU);
#endif

  /* Load the network. */
  net_.reset(new Net<float>(model_file, TEST));
  net_->CopyTrainedLayersFrom(weights_file);

  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

  Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  CHECK(num_channels_ == 3 || num_channels_ == 1)
    << "Input layer should have 1 or 3 channels.";
  input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

  /* Load the binaryproto mean file. */
  SetMean(mean_file, mean_value);
}

std::vector<vector<float> > Detector::Detect(const cv::Mat& img) {
  Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  Preprocess(img, &input_channels);

  net_->Forward();

  /* Copy the output layer to a std::vector */
  Blob<float>* result_blob = net_->output_blobs()[0];
  const float* result = result_blob->cpu_data();
  const int num_det = result_blob->height();
  vector<vector<float> > detections;
  for (int k = 0; k < num_det; ++k) {
    if (result[0] == -1) {
      // Skip invalid detection.
      result += 7;
      continue;
    }
    vector<float> detection(result, result + 7);
    detections.push_back(detection);
    result += 7;
  }
  return detections;
}

/* Load the mean file in binaryproto format. */
void Detector::SetMean(const string& mean_file, const string& mean_value) {
  cv::Scalar channel_mean;
  if (!mean_file.empty()) {
    CHECK(mean_value.empty()) <<
      "Cannot specify mean_file and mean_value at the same time";
    BlobProto blob_proto;
    ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

    /* Convert from BlobProto to Blob<float> */
    Blob<float> mean_blob;
    mean_blob.FromProto(blob_proto);
    CHECK_EQ(mean_blob.channels(), num_channels_)
      << "Number of channels of mean file doesn't match input layer.";

    /* The format of the mean file is planar 32-bit float BGR or grayscale. */
    std::vector<cv::Mat> channels;
    float* data = mean_blob.mutable_cpu_data();
    for (int i = 0; i < num_channels_; ++i) {
      /* Extract an individual channel. */
      cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
      channels.push_back(channel);
      data += mean_blob.height() * mean_blob.width();
    }

    /* Merge the separate channels into a single image. */
    cv::Mat mean;
    cv::merge(channels, mean);

    /* Compute the global mean pixel value and create a mean image
     * filled with this value. */
    channel_mean = cv::mean(mean);
    mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
  }
  if (!mean_value.empty()) {
    CHECK(mean_file.empty()) <<
      "Cannot specify mean_file and mean_value at the same time";
    stringstream ss(mean_value);
    vector<float> values;
    string item;
    while (getline(ss, item, ',')) {
      float value = std::atof(item.c_str());
      values.push_back(value);
    }
    CHECK(values.size() == 1 || values.size() == num_channels_) <<
      "Specify either 1 mean_value or as many as channels: " << num_channels_;

    std::vector<cv::Mat> channels;
    for (int i = 0; i < num_channels_; ++i) {
      /* Extract an individual channel. */
      cv::Mat channel(input_geometry_.height, input_geometry_.width, CV_32FC1,
          cv::Scalar(values[i]));
      channels.push_back(channel);
    }
    cv::merge(channels, mean_);
  }
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void Detector::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void Detector::Preprocess(const cv::Mat& img,
                            std::vector<cv::Mat>* input_channels) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}


using boost::asio::ip::tcp;
typedef boost::shared_ptr<tcp::socket> socket_ptr;

class SimpleTCPServer {
  public:
   SimpleTCPServer(const string &address, const string& port);
   bool read(uint8_t *buffer, size_t length);
   bool write(uint8_t *buffer, size_t length);

  private:
    void listenerThread();
    volatile bool talkerStarted;
    volatile size_t readBytes;
    volatile uint8_t * readBuffer;
    volatile size_t writeBytes;
    volatile uint8_t * writeBuffer;
    void talkerThread();
    boost::mutex readReadyMutex;
    boost::mutex writeReadyMutex;
    boost::mutex threadReadyMutex;
    boost::condition_variable_any readReady;
    boost::condition_variable_any writeReady;
    boost::condition_variable_any threadReady;
    boost::thread *listener;
    std::vector<boost::thread *> talkers;
    std::vector<socket_ptr> sockets;
    socket_ptr current_socket;
    string myaddress;
    string myport;
    
};



SimpleTCPServer::SimpleTCPServer(const string &address, const string& port)
{
  myaddress=address;
  myport=port;
  listener=new boost::thread(boost::bind(&SimpleTCPServer::listenerThread,this));
  readReadyMutex.lock();
  writeReadyMutex.lock();
}

bool SimpleTCPServer::read(uint8_t *buffer, size_t length)
{
   // tell tcp talkers how much we want read and wait for someone to fulfill our request
   readBuffer=(volatile uint8_t*)buffer;
   readBytes=length;
   while(readBytes!=0) {
      readReady.notify_one();
      readReady.wait(readReadyMutex);
   }
   return true;
}

bool SimpleTCPServer::write(uint8_t *buffer, size_t length)
{
   // tell tcp talkers how much we want written and wait for someone to fulfill our request
   writeBuffer=(volatile uint8_t*)buffer;
   writeBytes=length;
   while(writeBytes!=0) {
      writeReady.notify_one();
      writeReady.wait(writeReadyMutex);
   }
   return true;
}


void SimpleTCPServer::listenerThread()
{
  boost::unique_lock<boost::mutex> lock(threadReadyMutex);
  // Open port
  boost::asio::io_service io_service;
  try {
     tcp::resolver resolver(io_service);
     tcp::acceptor acceptor(io_service, tcp::endpoint(*tcp::resolver(io_service).resolve(tcp::resolver::query(myaddress,myport))));
     while (true) {
       socket_ptr sock(new tcp::socket(io_service));
       
       acceptor.accept(*sock);
       talkerStarted=false;
       current_socket=sock;
       sockets.push_back(sock);
       talkers.push_back(new boost::thread(boost::bind(&SimpleTCPServer::talkerThread,this)));
       
       while(!talkerStarted) {
          threadReady.wait(threadReadyMutex);
       }
     }
  } catch (std::exception& e) {
     std::cerr << "Exception: " << e.what() << "\n";
     exit(-1);
  }
}


void SimpleTCPServer::talkerThread() {
     socket_ptr mysocket;
     {
        boost::unique_lock<boost::mutex> lock(threadReadyMutex);
        mysocket=current_socket;
        talkerStarted=true;
        threadReady.notify_all();
     } // will remove the lock

     while (true) {
        
	{
		// we only do want to make ourselves available if there actually is data to be read
		uint8_t minibuffer[1];
		// do a blocking read
		try {
		   boost::asio::read(*mysocket,boost::asio::buffer(&minibuffer[0],1));
		} catch (std::exception& e) {
		}
		// now we know data is coming in, we can do an actual read, signal readyness
		
		boost::unique_lock<boost::mutex> readlock(readReadyMutex);
		while (readBytes==0) {
		   readReady.wait(readReadyMutex);
		}
		uint8_t * rb=(uint8_t*)(readBuffer+1);
		size_t length = readBytes-1;
		readBuffer[0]=minibuffer[0]; // copy the canary which has already been read over

		try {
		   boost::asio::read(*mysocket,boost::asio::buffer(rb,length));
		} catch (std::exception& e) {
		   mysocket->close();
		   while (true) {
		      readReady.wait(readReadyMutex);
		   }
		   // since read has not been acknowledged, another thread now can
		}
		//acknowledge succesful read
		readBytes=0;
                readReady.notify_all();
	}
	{
		boost::unique_lock<boost::mutex> writelock(writeReadyMutex);
		// after a read always a write follows
		while(writeBytes==0) {
		      readReady.wait(readReadyMutex);
		}

		uint8_t * wb=(uint8_t*)writeBuffer;
		size_t length = writeBytes;
		writeBytes=0; //acknowledge in all cases, since no other thread could do the write for us.
		try {
		   boost::asio::write(*mysocket,boost::asio::buffer(wb,length));
		} catch (std::exception& e) {
		   mysocket->close();
		   writeReady.notify_all();
		   while (true) {
		      writeReady.wait(writeReadyMutex);
		   }
		}
		writeReady.notify_all();
	}
     }
}


typedef struct __attribute__ ((__packed__)) {
	uint8_t label;
	float score;
	uint16_t xmin;
	uint16_t xmax;
	uint16_t ymin;
	uint16_t ymax;
} detection_info;

typedef struct __attribute__ ((__packed__)) {
	uint16_t count;
	detection_info detection[INT_MAX]; // over_allocated since variable length arrays are not allowed in C++
} detection_results;

#define IMAGESIZE 300
#define BUFFERLENGTH (IMAGESIZE*IMAGESIZE*3)

DEFINE_string(mean_file, "",
    "The mean file used to subtract from the input image.");
DEFINE_string(mean_value, "104,117,123",
    "If specified, can be one value or can be same as image channels"
    " - would subtract from the corresponding channel). Separated by ','."
    "Either mean_file or mean_value should be provided, not both.");
DEFINE_string(out_file, "",
    "If provided, store the detection results in the out_file.");
DEFINE_double(confidence_threshold, 0.01,
    "Only store detections with score higher than the threshold.");
DEFINE_string(network_address, "0.0.0.0",
    "Network address to bind to");
DEFINE_string(network_port, "9900",
    "Network port to bind to");

int main(int argc, char** argv) {
  ::google::InitGoogleLogging(argv[0]);
  // Print output to stderr (while still logging)
  FLAGS_alsologtostderr = 1;

#ifndef GFLAGS_GFLAGS_H_
  namespace gflags = google;
#endif

  gflags::SetUsageMessage("Do detection using SSD mode.\n"
        "Usage:\n"
        "    ssd_detect [FLAGS] model_file weights_file\n");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 3) {
    gflags::ShowUsageWithFlagsRestrict(argv[0], "examples/ssd/ssd_detect");
    return 1;
  }

  const string& model_file = argv[1];
  const string& weights_file = argv[2];
  const string& mean_file = FLAGS_mean_file;
  const string& mean_value = FLAGS_mean_value;
  const string& out_file = FLAGS_out_file;
  const string& address = FLAGS_network_address;
  const string& port = FLAGS_network_port;
  const float confidence_threshold = FLAGS_confidence_threshold;
  uint8_t inputBuffer[BUFFERLENGTH];

  // Initialize the network.
  Detector detector(model_file, weights_file, mean_file, mean_value);

  // Set the output mode.
  std::streambuf* buf = std::cout.rdbuf();
  std::ofstream outfile;
  if (!out_file.empty()) {
    outfile.open(out_file.c_str());
    if (outfile.good()) {
      buf = outfile.rdbuf();
    }
  }
  std::ostream out(buf);

  // Process image one by one.
  std::ifstream infile(argv[3]);
  std::string file;
  SimpleTCPServer server(address,port);
  while (server.read(inputBuffer,(size_t)BUFFERLENGTH)) {
      const int sizes[2]={IMAGESIZE,IMAGESIZE};
      cv::Mat img = cv::Mat(2,sizes,CV_8UC3,(void*)inputBuffer);
      std::vector<vector<float> > detections = detector.Detect(img);

      int num_detections = 0;
      detection_results *results=(detection_results*)inputBuffer;
      /* Print the detection results. */
      for (int i = 0; i < detections.size(); ++i) {
        const vector<float>& d = detections[i];
        // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
        CHECK_EQ(d.size(), 7);
        const float score = d[2];
        if (score >= confidence_threshold) {
          out << file << " ";
          out << static_cast<int>(d[1]) << " ";
          out << score << " ";
          out << static_cast<int>(d[3] * img.cols) << " ";
          out << static_cast<int>(d[4] * img.rows) << " ";
          out << static_cast<int>(d[5] * img.cols) << " ";
          out << static_cast<int>(d[6] * img.rows) << std::endl;
          results->detection[num_detections].label = static_cast<uint8_t>(d[1]);
          results->detection[num_detections].score = score;
          results->detection[num_detections].xmin = static_cast<uint16_t>(d[3] * img.cols);
          results->detection[num_detections].ymin = static_cast<uint16_t>(d[4] * img.rows);
          results->detection[num_detections].xmax = static_cast<uint16_t>(d[5] * img.cols);
          results->detection[num_detections].ymax = static_cast<uint16_t>(d[6] * img.rows);
          num_detections++;
        }
      }
      results->count = num_detections;
      size_t resultsize = offsetof(detection_results,detection[0])+num_detections*sizeof(detection_info);
      server.write(inputBuffer,(size_t)resultsize);
      out << "wrote " << num_detections << " detections in " << resultsize << " bytes" << std::endl;
  }
  return 0;
}
#else
int main(int argc, char** argv) {
  LOG(FATAL) << "This example requires OpenCV; compile with USE_OPENCV.";
}
#endif  // USE_OPENCV
