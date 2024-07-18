# 最后要实现下面这个原来launch file的效果

1.roslaunch stretch_gazebo gazebo.launch rviz:=true

2.roslaunch da_sim single_goal_ctrl.launch
<launch>
    <node name="teleop_controller" pkg="da_core" type="teleop_controller.py" respawn="true"/>
    <node name="kb_interface" pkg="da_interface" type="keyboard_interface.py" output="screen" required="true"/>
    <node name="goal_manager" pkg="da_sim" type="single_goal_manager.py" required="true" />
</launch>