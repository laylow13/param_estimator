## 功能介绍
基于EKF，用于无人机的在线参数（状态）估计程序，使用PX4 1.14作为底层飞行栈。
- lib中含有一个自定义的EKF算法和matlab coder生成的EKF库。
- src中为封装后的ROS2估计器节点，以及一个理想系统的在线估计测试例程。
- 估计器节点订阅PX4的执行器输出话题，加速度，角速度，姿态等话题，输出计算出的推力力矩，估计的参数（状态）等。
