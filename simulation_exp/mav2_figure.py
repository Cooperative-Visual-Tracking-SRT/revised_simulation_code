import rosbag
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

bag = rosbag.Bag("./mav2_99loss_random.bag")

N = bag.get_message_count()
start_t = bag.get_start_time()
end_t = bag.get_end_time()

times = np.zeros((N,1))
gt_x = np.zeros((N,1))
gt_y = np.zeros((N,1))
mav1_x = np.zeros((N,1))
mav1_y = np.zeros((N,1))
mav2_x = np.zeros((N,1))
mav2_y = np.zeros((N,1))
error_x = np.zeros((N,1))
error_y = np.zeros((N,1))

step = 0
for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
    for cnt in range(len(msg.name)):
        if msg.name[cnt]=="actor":
            idx = cnt
            break
    gt_y[step] = msg.pose[idx].position.x
    gt_x[step] = msg.pose[idx].position.y
    step += 1

print("done")

step = 0
for topic, msg, t in bag.read_messages(topics=['/machine_1/target_tracker/pose']):
    times[step] = t.to_time()
    mav1_x[step] = msg.pose.pose.position.x
    mav1_y[step] = msg.pose.pose.position.y
    error_x[step] = np.abs(mav1_x[step]-gt_x[step])
    error_y[step] = np.abs(mav1_y[step]-gt_y[step])
    step += 1
print("done")

step = 0
for topic, msg, t in bag.read_messages(topics=['/machine_2/target_tracker/pose']):
    mav2_x[step] = msg.pose.pose.position.x
    mav2_y[step] = msg.pose.pose.position.y
    error_x[step] = max(error_x[step],np.abs(mav2_x[step]-gt_x[step]))
    error_y[step] = max(error_y[step],np.abs(mav2_y[step]-gt_y[step]))
    step += 1
print("done")

mu_x = np.mean(error_x)
mu_y = np.mean(error_y)
sigma_x = np.std(error_x)
sigma_y = np.std(error_x)
plt.rcParams['figure.figsize'] = (12.0, 6.0)
plt.figure()
plt.subplot(2,1,1)
plt.plot(times[np.nonzero(times)],mav1_x[np.nonzero(times)],label="mav1")
plt.plot(times[np.nonzero(times)],mav2_x[np.nonzero(times)],label="mav2")
plt.plot(times[np.nonzero(times)],gt_x[np.nonzero(times)],label="gt")
plt.plot(times[np.nonzero(times)],error_x[np.nonzero(times)],label="error")
plt.legend(prop={'size':9})
plt.yticks([2.5*i for i in range(-5,6)])
plt.xticks([10*i for i in range(5,18)])
plt.grid()
plt.xlabel("simulation time/s")
plt.ylabel("pose estimation & error x/m")
plt.title("2 drones, 99 communication loss, mean error ({:.2f},{:.2f}), standard error ({:.2f},{:.2f})".format(mu_x,mu_y,sigma_x,sigma_y))

plt.subplot(2,1,2)
plt.plot(times[np.nonzero(times)],mav1_y[np.nonzero(times)],label="mav1")
plt.plot(times[np.nonzero(times)],mav2_y[np.nonzero(times)],label="mav2")
plt.plot(times[np.nonzero(times)],gt_y[np.nonzero(times)],label="gt")
plt.plot(times[np.nonzero(times)],error_y[np.nonzero(times)],label="error")
plt.legend(prop={'size':9})
plt.yticks([2.5*i for i in range(0,9)])
plt.xticks([10*i for i in range(3,18)])
plt.grid()
plt.xlabel("simulation time/s")
plt.ylabel("pose estimation & error y/m")
plt.show()
    
