chassis
```
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host registry.cn-hangzhou.aliyuncs.com/fishros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
```

laser

```
xhost + && sudo docker run -it --rm -p 8889:8889 -p 8889:8889/udp -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/snd -e DISPLAY=unix$DISPLAY registry.cn-hangzhou.aliyuncs.com/fishros/fishbot_laser
```

save map
```
ros2 run nav2_map_server map_saver_cli -t map -f fishbot_map
```

bringup
```
ros2 launch fishbot_bringup bringup_quick.launch.py
```

launch navigation

```
ros2 launch fishbot_navigation2 navigation2.launch.py use_sim_time:=False
```