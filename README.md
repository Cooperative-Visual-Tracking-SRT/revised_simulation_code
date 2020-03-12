# Description

此仓库为 robot-perception-group 的AIRCAP项目仿真代码基于ubuntu18和gazebo9更改后的版本，包括更改后的caffe和rotors_simulator

## Preparation

将这两个包以及 https://github.com/robot-perception-group/AIRCAP 描述所需的所有的包均放到~/catkin_ws/src下

## caffe

由于环境不同，caffe需要重新编译

Note:

* 之前编译环境为gcc7.4，python2.7，cuda10.1和cudnn7.6.5，显卡为Nvidia 1060M
* 需要ubuntu18下ros-melodic-desktop-full及其所有依赖包，建议所有包都通过apt安装

重新编译方法（时间比较长）：

    cd ~/catkin_ws/src/caffe
    make clean
    make all -j8
    make pycaffe
    make test -j8
    (optional) make runtest -j8
    
编译完成后将caffe/python加入PYTHONPATH环境变量：
    
    export PYTHONPATH=$PYTHONPATH:~/catkin_ws/src/caffe/python
    source ~/.bashrc

运行测试：
    
    cd ~/catkin_ws/src/caffe
    python2 examples/ssd/ssd_pascal_webcam.py

如果笔记本电脑有摄像头的话会打开摄像头，对摄像头获取的图像进行检测

## rotors_simulator

只需要放到~/catkin_ws/src下即可

## catkin_make

Note:

* 在所有ros包克隆到catkin_ws后，安装必要依赖包

  运行
  
      rosdep install --from-paths ~/catkin_ws/src --ignore-src

* catkin_make检查没有安装的软件包，名字比较奇怪的大多是ros包，解决方法如下

  例如提示找不到pose-cov-ops，运行

      sudo apt-get install ros-melodic-pose-cov-ops
  
  若其他如boost等包报错，如“未定义的引用”等，请确保所有包都是通过apt安装的；如果google不能解决，那么最快的解决方法是重装系统
      
* 有些包会和anaconda里面的包起冲突，例如boost，并没有很好的解决方案。我的解决方法是把anaconda卸了（

* 注意在整个过程中需要首先保证ros melodic环境已经初始化

      source /opt/ros/melodic/setup.bash

利用catkin编译所有包。运行

    cd ~/catkin_ws
    catkin_make

编译是否通过很依赖环境，如果编译通过则进行下一步。

## run simulation

确认自己系统有没有安装screen，运行

    sudo apt-get install screen

如果没有用过screen，请参考教程 https://www.cnblogs.com/mchina/archive/2013/01/30/2880680.html

确认有没有把编译好的包初始化，运行

    source ~/catkin_ws/devel/setup.bash

最简单的运行仿真方式为

    cd ~/catkin_ws/src/scrips/simulation/
    bash ./setup_mavocap_gazebo.sh <number of robots> <communication success rate> <experiment title>

运行以上命令后，命令行上有提示仿真进行到哪一步。由于是用screen在后台运行，都运行完成后主终端看起来什么都没发生。这时候运行

    # 查看所有session
    screen -ls
    # 进入某一session查看运行状态
    screen -r session_number.session_name
    
    # 如果想要关掉所有session重新开始，运行以下命令
    # <number of sessions>换成session个数，也可以直接指定一个大整数如20
    screen -ls|awk 'NR>=1&&NR<= <number of sessions> {print $1}'|awk '{print "screen -S "$1" -X quit"}'|sh

进入每一个session看看到底是哪个没有运行成功，再进一步debug。一种等价的方法是，打开脚本setup_mavocap_gazebo.sh逐句运行找bug。

## simulation experiment

在IROS-2018论文中，原作者Price等人利用gazebo跑了三个仿真实验，若需要复现，或是需要利用仿真测试移植后的系统性能可以按照以下说明进行。

首先按照 https://github.com/robot-perception-group/AIRCAP 中复现论文仿真实验的说明修改setup_mavocap_gazebo.sh。因为需要实验数据记录，需要创建一个记录文件夹（例如/home/user/log）并写在以上sh文件的LOGPATH变量中，然后将文件接近末尾关于rosbag数据记录的代码取消注释，也就是

    #start logs
    echo "Starting recording..."
    echo screen -d -m -S ROSBAG bash -i -c "rosbag record -o ${LOGPATH}/${NAME}.bag $( cat bagtopics.txt | tr '\n' ' ' )"
    screen -d -m -S ROSBAG bash -i -c "rosbag record -o ${LOGPATH}/${NAME}.bag $( cat bagtopics.txt | tr '\n' ' ' )"

    echo "Running experiment for 120 seconds"
    ./rossleep.py 120
    
以上代码会自动在LOGPATH中创建rosbag文件，记录bagtopics.txt中写到的所有topic。打开bagtopics.txt文件，在最后加入一行/gazebo/model_states，这样可以不用原作者的target_publish节点获得被跟踪目标的ground_truth位置坐标。

仿真环境中actor默认在两点一线之间移动。原作者似乎写了一些其他方式移动的代码，但是用起来有bug；其实可以用一种非常土的方法实现actor路径的规划：仿真利用target_RAL.sdf这个文件生成actor，只需要修改actor中的描述，设定一些时间节点和waypoint，就能够实现actor按照路径规划的移动。可以利用simulation_exp文件夹中的random_traj.py文件生成随机waypoint的xml代码，之后将其复制粘贴到target_RAL.sdf中（详见simulation_exp文件夹中的target_RAL.sdf）。由于仿真时间是有限的，只要waypoint足够多时间足够长，就可以认为actor的移动处于伪随机的状态。

仿真计时120s，仿真结束后kill掉screen的所有session即可。此时LOGPATH中会生成*.bag.active的文件，运行以下命令使其转化为bag文件：

    rosbag reindex *.bag.active
    rosbag fix *.bag.active <name you want>.bag

转化为bag文件后可以利用仓库simulation_exp文件夹中的作图代码mav2_figure.py画出目标坐标ground_truth和无人机对目标的detection随时间变化的情况，并计算误差的均值和标准差。（由于没怎么用过rosbag和matplotlib，作图代码写的非常丑陋qaq）

最后，在simulation_exp/figures文件夹中存储了2架无人机、不同loss下的单次实验（没有取平均）结果，可以大概展现实验的估计精度并体现communication loss的影响。

# Acknowledgement

非常感谢 robot-perception-group 的AIRCAP项目提供的代码，此仓库中的代码为源代码基于ubuntu18和gazebo9环境的移植。

# Annex 

源代码git链接
* catkin_ws其他必要包和构建包的详细流程：https://github.com/robot-perception-group/AIRCAP
* rotors_simulator源代码（for gazebo 8）：https://github.com/robot-perception-group/rotors_simulator

其他
* 编译好的catkin_ws、caffe和rotors_simulator清华云盘链接：https://cloud.tsinghua.edu.cn/d/8505c576b2ab430aadd9/
* 渣像素仿真运行成果效果（6架无人机）

![](https://github.com/Cooperative-Visual-Tracking-SRT/revised_simulation_code/blob/master/result.jpg)
