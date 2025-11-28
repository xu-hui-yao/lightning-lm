conda deactivate
source /opt/ros/jazzy/setup.bash  # 配置ros2环境
rm -rf bin build install log  # 清除之前构建的
colcon build  # 构建
source install/setup.bash  # 绑定命令到终端
# ros2 run lightning run_slam_offline --config ./config/default_vbr.yaml --input_bag /home/xuhuiyao/Desktop/workfield/data/VBR/campus/ros2.db3
# ros2 run lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag /home/xuhuiyao/Desktop/workfield/data/NCLT/20120115/20120115.db3
ros2 run lightning run_slam_offline --config ./config/default_livox.yaml --input_bag /home/xuhuiyao/Desktop/workfield/ros2_bag/test1/test1_0.db3
