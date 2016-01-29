FROM ros:indigo
#place here your application's setup specifics

#apt-get update only needs to be run the first time you build the image to
#update the repo that contains build-essential won't be updated
RUN apt-get update
RUN apt-get install build-essential ros-indigo-cv-bridge -y

#Note: RUN apt-get upgrade fails because it creates too many parent processes(?)
#Apparently a Dockerfile can only accept 127 RUN statements

EXPOSE 11311
ADD rostesting /home/rostesting
ADD eigen-eigen-b30b87236a1b /home/eigen

ENV ROS_ROOT=/opt/ros/indigo/share/ros
ENV ROS_PACKAGE_PATH=/home/rostesting/rover_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
ENV ROS_MASTER_URI=http://localhost:11311
ENV LD_LIBRARY_PATH=/home/rostesting/rover_ws/devel/lib:/home/rostesting/rover_ws/devel/lib/x86_64-linux-gnu:/opt/ros/indigo/lib/x86_64-linux-gnu:/opt/ros/indigo/lib
ENV CATKIN_TEST_RESULTS_DIR=/home/rostesting/rover_ws/build/test_results
ENV CPATH=/home/rostesting/rover_ws/devel/include:/opt/ros/indigo/include
ENV ROS_TEST_RESULTS_DIR=/home/rostesting/rover_ws/build/test_results
ENV PATH=/home/rostesting/rover_ws/devel/bin:/opt/ros/indigo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV ROSLISP_PACKAGE_DIRECTORIES=/home/rostesting/rover_ws/devel/share/common-lisp
ENV ROS_DISTRO=indigo
ENV PYTHONPATH=/home/rostesting/rover_ws/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages
ENV PKG_CONFIG_PATH=/home/rostesting/rover_ws/devel/lib/pkgconfig:/home/rostesting/rover_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/indigo/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/indigo/lib/pkgconfig
ENV CMAKE_PREFIX_PATH=/home/rostesting/rover_ws/devel:/opt/ros/indigo
ENV ROS_ETC_DIR=/opt/ros/indigo/etc/ros

WORKDIR /home/rostesting/rover_ws

#Manually install the eigen lib
RUN cmake /home/eigen; make install

#builds the ros project and enables ros commands through the cli
#RUN /opt/ros/indigo/bin/catkin_make
#RUN source devel/setup.bash


#RUN apt-get update && apt-get install -y \
#    ros-indigo-ros-tutorials \
#    ros-indigo-common-tutorials \
#    && rm -rf /var/lib/apt/lists/
CMD [ "roslaunch", "my-ros-app my-ros-app.launch" ]

#when you are in the workspace (rover_ws), run: catkin_make
#In the same directory run source devel/setup.bash
#In order to run ros commands in a bash terminal run: source /opt/ros/indigo/setup.bash
