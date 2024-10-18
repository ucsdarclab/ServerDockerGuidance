# Using Ros2 with docker [^1]

### Pull from a pre-built Dockerfile on the internet:

```
From rwthika/ros2-torch

RUN sudo apt update
RUN sudo apt install -y ros-humble-turtlesim
RUN sudo apt install -y '~nros-humble-rqt*'
RUN sudo rosdep update
RUN sudo apt install -y ros-humble-rviz2
RUN sudo apt install -y tmux

# extra packages
ENV TORCH_CUDA_ARCH_LIST="Turing Ampere"
RUN export FORCE_CUDA=1 && pip install "git+https://github.com/facebookresearch/pytorch3d.git"
RUN pip install \
 --ignore-installed \
 open3d==0.18.0 \
 numpy==1.26.4 \
 tetgen==0.6.4 \
 pyvista==0.43.5 \
 pyacvd==0.2.10
```
This comes with pytorch and cuda. Other variants can be found in this [page](https://github.com/ika-rwth-aachen/docker-ros-ml-images/tree/main).
### Build image

```
docker build -t ros2-torch .
```

### Start container

```
xhost +local:

docker run -it --rm \
    --env DISPLAY=:0 \
    --net=host \
    --gpus all \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume <your_worksapce>:/docker-ros/ws \
    --env DOCKER_UID=$(id -u) \
    --env DOCKER_GID=$(id -g) \
    --privileged \
    --name ros2-torch \
    ros2-torch 
```
This script will enable x11 forwarding.

[^1]: Last update: 10/18/2024, by Xiao Liang 
