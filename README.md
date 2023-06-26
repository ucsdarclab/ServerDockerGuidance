## ARCLab Server Instruction [^1]


This is a very simple guide for using servers and docker. All techniques introduced here are pretty simple. If later you find advanced techniques that you think can be a good extension of this file, please let the maintainer know!!!

**First thing: you should not directly install any packages or change system setting on the lab servers. All dependencies and environments should be properly maintained in DOCKER.** A common workflow of docker is 1.building a **image** with some configuration file, 2. run a **container** from a built image.  


#### Build Docker Image

In general, a docker image can be build by a `Dockerfile` file. Here is a simple example Dockerfile (for pytorch environment):

```
FROM pytorch/pytorch:1.9.1-cuda11.1-cudnn8-runtime  # inherit an existing image from dockerhub

RUN apt-get update 
RUN apt-get install -y build-essential              # linux packages, usually useful for c++ applications

RUN mkdir /path/<...>                               # make a directory

COPY . /opt/app                                     # copy the current directory (of Dockerfile) to /opt/app on virtual file system
WORKDIR . /opt/app                                  # sets the working directory for any later docker commands

RUN pip install -r requirements.txt                 # install python dependency with -r. The requirements.txt file should be in the same dir as Dockerfile.
RUN pip install jupyter                             # you can also directly install from using pip
```

In case you would like to use mamba (or conda) to manage environment in docker:

```
FROM nvcr.io/nvidia/cuda:11.3.0-cudnn8-runtime-ubuntu20.04

#Obtain curl
RUN apt-get -y update
RUN apt-get -y install -y curl

#Obtain Micromamba
RUN curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh" \
    && bash Mambaforge-$(uname)-$(uname -m).sh -bfp /usr/local

#Run MicroMamba commands
WORKDIR /usr/src/app
COPY resources/env.yml ./env.yml 
COPY resources/requirements.txt ./requirements.txt
RUN mamba env update create -f env.yml
SHELL ["mamba","run","-n","ndg","/bin/bash","-c"]
SHELL ["/bin/bash","-c"]
RUN mamba init
RUN echo 'mamba activate <env_name>' >> ~/.bashrc
WORKDIR /home
```

After you have customized your Dockerfile, you can then run `docker build -t <image_name> /path/to/Dockerfile`. You can check existing docker images with `docker image ls`. Please make sure you remove all `<none>` image if those are results of your build.


#### Run Docker Container

Once you have a docker image, you can run a container on top of that image. An example command is

```
docker run --rm  -p <port>:<port> -it --gpus '"device=0"' --shm-size 16G -v /path/on/server:/path/in/container --name <container_name> <image_name> bash
```

```
-p       for port forwarding between server machine and the container. You can use multiple -p flag for multiple ports.
-gpus    for selecting gpus
-v       for connecting (mounting) filesystem of the server to that of the container. Multiple -v if you want to mount mutltiple src:dest pairs.
bash     this keyword will open a bash shell of the container for you after you run the 'docker run' command.
```

To exit the container, simply type `ctrl c+d` (this will kill the container if you have `--rm` tag). You can exam all running containers with `docker container ls`. 

A container might be lost if it stays idle for a long time. To prevent your training job from stopping, a solution is to use the `screen` command on linux. After entering into a screen, a container won't be killed unless you command it. It is a good practice to save important data to a mounted volume specified with `-v`. Data that are not write to the shared filesystem will be deleted after the container stops.

[^1]: Last update: 5/18/2023, by Xiao Liang 
