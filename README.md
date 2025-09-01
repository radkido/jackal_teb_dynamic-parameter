# jackal_teb_dynamic-parameter

teb参数文件位置：jackal/jackal_navigation/params/teb_local_planner_params.yaml

# 启动gazebo仿真

roslaunch jackal_gazebo jackal_world.launch config:=front_laser

# 启动movebase gmapping+teb 

roslaunch jackal_navigation gmapping_demo.launch 

# 调参

python3 你的絕對路徑/complete.py
