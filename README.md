# jackal_teb_dynamic-parameter



#启动gazebo仿真

roslaunch jackal_gazebo jackal_world.launch config:=front_laser

#启动movebase gmapping+teb 

roslaunch jackal_navigation gmapping_demo.launch 

#调参

python3 src/jackal/jackal_navigation/launch/complete.py
