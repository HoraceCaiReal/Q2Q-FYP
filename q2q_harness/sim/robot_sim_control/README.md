# robot_sim_control

仅为Fortress/gz_ros2_control 0.7.21增加可版本化的硬件位置增益入口，不修改系统安装，也不复制整个控制后端。

原插件主关节位置误差乘`.1 × update_rate`转速度，mimic误差乘`update_rate`，两侧闭环刚度不同；参数在initSim缓存。适配器先从URDF硬件参数设定增益，再调用原实现。

默认`.1`使用原插件；`step3.ps1 -PositionGain 1`显式启用适配器，使50Hz更新时主关节与mimic均约50/s。该值必须配合轨迹斜坡、有限物理步长和独立回归，不宣称为实机控制增益或力控制。
