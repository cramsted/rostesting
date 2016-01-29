NAME = ros
VERSION = indigo
PORTS = 8080:8080


test:
	docker build -t $(NAME):$(VERSION) .
	docker run --rm --name $(NAME)-test -t -i $(NAME):$(VERSION)  /bin/bash

run:
	docker run --rm --name $(NAME)-test -t -i $(NAME):$(VERSION)  /bin/bash

ros-tut:
	docker build --tag ros:ros-tutorials .
	docker network create foo
	docker run -it --rm \
    --net foo \
    --name master \
    ros:ros-tutorials \
    roscore
	docker run -it --rm \
    --net foo \
    --name talker \
    --env ROS_HOSTNAME=talker \
    --env ROS_MASTER_URI=http://master:11311 \
    ros:ros-tutorials \
    rosrun roscpp_tutorials talker
	docker run -it --rm \
    --net foo \
    --name listener \
    --env ROS_HOSTNAME=listener \
    --env ROS_MASTER_URI=http://master:11311 \
    ros:ros-tutorials \
    rosrun roscpp_tutorials listener

t:
	docker build --tag ros:ros-tutorials .
	docker run --rm --name ros-tutorials-test -t -i ros:ros-tutorials  /bin/bash
