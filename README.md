更改后的仿真代码，包括更改后的caffe和rotors_simulator

## Preparation

将这两个包以及 https://github.com/robot-perception-group/AIRCAP 描述所需的所有的包均放到~/catkin_ws/src下

## caffe

由于环境不同，caffe需要重新编译

Note:

* 之前编译环境为gcc7.4，python2.7，cuda10.1和cudnn7.6.5
* 需要ubuntu18下ros-melodic-desktop-full及其所有依赖包，建议所有包都通过apt安装

重新编译方法（时间比较长）：

    cd ~/catkin_ws/caffe
    make clean
    make all -j8
    make pycaffe
    make test -j8
    (optional) make runtest -j8
    
编译完成后将caffe/python加入PYTHONPATH环境变量：
    
    export PYTHONPATH=$PYTHONPATH:~/catkin_ws/caffe/python
    source ~/.bashrc

运行测试：
    
    cd ~/catkin_ws/caffe
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

利用catkin编译所有包。运行

    cd ~/catkin_ws
    catkin_make

编译是否通过很依赖环境，如果编译通过则进行下一步。

## run simulation

确认自己系统有没有安装screen，运行

    sudo apt-get install screen

如果没有用过screen，请参考教程 https://www.cnblogs.com/mchina/archive/2013/01/30/2880680.html

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

# Acknowledgement

非常感谢 robot-perception-group 的AIRCAP项目提供的代码，此仓库中的代码为源代码基于ubuntu18和gazebo9环境的移植。

# Annex

源代码git链接
* catkin_ws其他必要包和构建包的详细流程：https://github.com/robot-perception-group/AIRCAP
* rotors_simulator源代码（for gazebo 8）：https://github.com/robot-perception-group/rotors_simulator

其他
* 编译好的catkin_ws、caffe和rotors_simulator清华云盘链接：https://cloud.tsinghua.edu.cn/d/8505c576b2ab430aadd9/
