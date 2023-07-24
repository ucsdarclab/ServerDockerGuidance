## Omniverse/Isaac Sim Documentation [^1]

The following is meant to serve as an update to the ARCLab server's [Docker Guide](https://github.com/ucsdarclab/ServerDockerGuidance/blob/main/README.md) describing the installation process of **NVIDIA Omniverse/Isaac Sim** and one method of creating a containerized environment to run Isaac Sim off the lab's server.

Same as with the Docker Guide, there are other techniques/methods which may be a good extension of this document or may be more efficient. Feel free to update this page with the prevalent information.

### Prerequisite(s):
Complete the necessary environment setup on the [Docker Guide](https://github.com/ucsdarclab/ServerDockerGuidance/blob/main/README.md).

Summary:
1. Install Docker
2. Create a work environment within the lab's server
### On Local Machine: [(NVIDIA Installation Documentation)](https://docs.omniverse.nvidia.com/isaacsim/latest/install_workstation.html)

#### Installation: Omniverse Launcher, Omniverse Streaming Client
The **Omniverse Launcher** can be used to launch Isaac Sim directly (assuming Isaac Sim is to be launched from your local maching and ***NOT*** from a server). It is also needed to download and launch the **Omniverse Streaming Client** which will later be used to stream from the remote container to your local machine.
1. Download the [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/download/)
    * This will require you to create your own NVIDIA account if you have not done so already.
    * Once installation is complete, open the Omniverse Launcher application, complete the installation process, and log in using your NVIDIA account information.
2. Install the [Omniverse Streaming Client](https://docs.omniverse.nvidia.com/streaming-client/104.0.0/user-manual.html)
    * From the ***Exchange*** tab, search "Omniverse Streaming Client", select, and install the application.
    * Launch the application from the ***Library*** tab.

### On Remote Machine:

#### Installation: NVIDIA Dockerfile(s)
In substitute of downloading **Isaac Sim** locally, the application may be launched via Dockerfile in its own container.
1. Follow the instructions on the [Isaac Sim github](https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles) to get access to clone the repository. 
    * Get access to the [Isaac Sim Container](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim), and either generate an [NGC API Key](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key) or reference a currently existing one (assuming you've generated one previously)
2. Build or reference the image on the ARCLab server.
    * Log onto the server and navigate the path to where you want to clone NVIDIA's Isaac Sim repository.
    * Cloning the Isaac Sim repository requires access. <br/>Run:<br/>
        ```
        docker login nvcr.io
        ```
    Use **$oauthtoken** in place of username <br/>
    Use your generated **NGC API key** as the password (copy and paste) <br/>
    * Clone the [Isaac Sim Container](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)
    * Build the image (the original image has been deleted from the ARCLab server, check that it has not been recreated before building):<br/>
        ```
        docker build --pull -t \
            isaac-sim:2022.2.1-ubuntu20.04 \
            --build-arg ISAACSIM_VERSION=2022.2.1 \
            --build-arg BASE_DIST=ubuntu20.04 \
            --build-arg CUDA_VERSION=11.4.2 \
            --build-arg VULKAN_SDK_VERSION=1.3.224.1 \
            --file Dockerfile.2022.2.1-ubuntu20.04 
        ```
3. Create a ```.sh``` file within the cloned repository, paste the following within the file:
    ```
    docker run --name isaac-sim --entrypoint bash -it --gpus 1 -e "ACCEPT_EULA=Y" --rm \
        -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit:rw \
        -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
        -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
        -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
        -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
        -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
        -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
        -v ~/docker/isaac-sim/documents:/root/Documents:rw \
        isaac-sim:2022.2.1-ubuntu20.04
    ```
    * Running the ```.sh``` file will bring up a bash terminal with all the associated files necessary for running Isaac Sim.
    * You may verify that the container is running properly by entering ```docker ps``` into a separate terminal connected to the server. Check that 'isaac-sim' is one of the running containers.
4. Before running the Isaac Sim application (but after running the ```.sh``` file where you should be in the container's bash terminal), it would be best to generate the IP address associated with the container (By default, this is identical to the server's IP) <br/>
Run the following in the container's bash terminal:
    ```
    hostname -I | awk '{print $1}'
    ```
    * Copy the IP address as it will be necessary to connect via the streaming client. 
    * **Note:** This is necessary because of the ```--network=host``` flag that is part of the command copied to the ```.sh``` file. If you don't want to run the container on the ```host``` network then you will have a different IP address.

5. In the **/isaac-sim/** directory, run in headless mode:
    ```
    ./runheadless.native.sh
    ```
    * Isaac Sim is now running.
6. Generate the UI by connecting to the container through the **Omniverse Streaming Client** 
    * If possible, you will want to open the Omniverse Streaming Client in another desktop as the GUI will fill the entire screen and is non responsive when exited. <br> <br/>
    First, open the secondary desktop and then open the steaming application through the **Omniverse Launcher**.

    * Paste the server's IP address into the first box and select your screen's resolution

### Exiting The Streaming Client

Exiting the streaming client is unintuitive as of the current build.
1. Within the GUI, select: File < Exit
    * This will begin to shutdown the GUI, but usually will only get to an error popup stating that the application is unresponsive.
    * Select ***Close***.

    Note: An alternate method of shutting down the GUI is by killing the task within ***Task Manager***, though you may still have to select ***Close*** due to the 'unresponsive' error.
2. Close the container by exiting out of the bash terminal.
* ***Note that all applications saved wtihin the GUI will be persisted after the container has been closed, HOWEVER, all files, folders, and changes to any of the Python source code will not be saved if done from a separate terminal.***

### Workflow Suggestions:
The following will outline the general workflow that I work with. The second terminal will serve as your main method of altering the source code of each simulation.

**Edit:** These suggestions are corroberated by [NVIDIA's FAQ](https://docs.omniverse.nvidia.com/isaacsim/latest/install_faq.html#isaac-sim-setup-net-host). 
1. Open a minimum of two bash shell terminals connected to the running container.
    * Open another terminal, connect to the server, and navigate to the cloned NVIDIA directory.
    * After starting the container's initial terminal, you can connect another bash terminal using:
        ```
        docker exec -it <name_of_container> bash
        ```
    Note: You can check the list of running container names using:
        ```docker ps```

2. Install vim or nano on the server to alter the source code and/or access the hidden files that allow for ssh tunneling.

3. Save any changes (such as the text editors or any changes to the file structure) made to the current container by doing the following:
    * Sign onto the ARCLab server using another terminal. Input:
        ```
        docker commit <current_container_name> <new_image_name:tag>
        ```
    * ```current_container_name``` is the name of a **running** container.
    * ```new_image_name``` is the name of the image which contains the saved changes. ```tag``` is simply the tag associated with the new image.
    * Delete the outdated image using:
        ```
        docker rmi <old_image:tag>
        ```
    * Update the ```.sh``` file used to run the container with the new image name (assuming the updated image has a different name). 

[^1]: Last update: 7/24/2023, by Cash Rich