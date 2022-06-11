# 실행은 하기 위해서는 총 3개의 터미널을 열어야함.

## roscore 서버 올리기
``` bash
cd catkin_ws
roscore
```

## server node실행
``` bash
source ~/.bashrc
source devel/setup.bash
rosrun mobinn_BOT_node mobinn_BOT_node
```

## pub node실행
``` bash
source ~/.bashrc
source devel/setup.bash
rosrun mobinn_SERVER_node mobinn_SERVER_node
```