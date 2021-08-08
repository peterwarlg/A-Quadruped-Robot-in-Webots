# A-Quadruped-Robot-in-Webots

### 软件

目前只支持webots R2019a版本，即第一个开源版本

### 关节

关节（电机）设置为力控，在此基础上手动封装一层PID，可以使用位控模式

### 运动学

* 对于单腿，只具有三个自由度，直接采用数学解法解得运动学逆解，不采用DH方式
* 姿态控制：
  * 世界坐标建立在机身水平时的中心位置
  * 机身坐标在机身中心，与机身固连
  * 姿态逆解求法为： 机身相对世界的RPY偏转 →机身相对足端的偏转(RPY+XYZ) →髋关节相对足端的偏移（XYZ） →求出足端相对髋关节的位置（x,y,z）	

* 整体运动采用步态规划

### 下一步工作

* 在目前的动作上扩展，实现全方位的运动以及姿态控制
* 使用DRL进行运动学习

### 其他备注

只定义physics不定义boundingObject会报错。将原有的servo节点改为hingejoint节点，在hingejoint节点àdevice节点新增motor和position sensor，因为servo节点无法读取位置.

Hingejoint下三个子节点：

第一个<jointparameters>定义转动轴位置与方向，jointparameters 默认不存在，需要双击新增。jointparameters 下属position是当前的转角而不是在世界中的位置，axis定义转动轴方向，anchor定义的才是转动轴在空间中的位置。

第二个device可添加电机（rotation motor）（参考referenceàNodes and APIàmotor）和位置传感器（position sensor）（参考referenceàNodes and APIàposition sensor）和制动器（brake），位置控制只用定义rotation motor。注意定义motor的name属性和程序的对应。现在也有servo这个节点，但是不在base nodes，在device—genetic—servo，servo不可读取位置，因此用新增motor之后再添加position sensor

第三个 endpoint，一般是新增一个soild,可以理解为末端执行器。然后在这个节点下继续建模。

Friday Robot和Tuesday Robot腿部关节不同，Friday也多一个力矩控制的模式，但是没有加上重力等作用力的补偿，膝关节为位置控制，髋关节两个电机是力控。关于这个referenceàNodes and APIàmotor的函数介绍也说了一些力矩控制的，具体看wb_motor_set_torque这个函数的说明。

这两个可以用来实验新步态，程序里做了很多注释。