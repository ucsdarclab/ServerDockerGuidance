# Install Curobo with Issac Sim [^1]
This guide provides step-by-step instructions for running NVIDIA’s Isaac Sim remotely from the lab environment with curobo.
## Prerequisites
- UCSD Wifi or VPN
- NGC account for accessing NVIDIA containers (opitonal if docker already pulled)
## Steps
1. **Clone curobo repo**
   ```
   git clone https://github.com/NVlabs/curobo.git
   ```
   ```cd curobo/docker``` into the directory.
   Replace content in ```docker_build.sh``` with
   ```
   #!/bin/bash
   ##
   ## Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
   ##
   ## NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
   ## property and proprietary rights in and to this material, related
   ## documentation and any modifications thereto. Any use, reproduction,
   ## disclosure or distribution of this material and related documentation
   ## without an express license agreement from NVIDIA CORPORATION or
   ## its affiliates is strictly prohibited.
   ##
   # This script will create a dev docker. Run this script by calling `bash build_dev_docker.sh`
   # If you want to build a isaac sim docker, run this script with `bash build_dev_docker.sh isaac`
   # Check architecture to build:
   image_tag=“x86”
   isaac_sim_version=“”
   input_arg=“$1”
   if [ -z “$input_arg” ]; then
      arch=$(uname -m)
      echo “Argument empty, trying to build based on architecture”
      if [ “$arch” == “x86_64” ]; then
         input_arg=“x86"
      elif [ “$arch” == “arm64" ]; then
         input_arg=“aarch64”
      elif [ “$arch” == “aarch64” ]; then
         input_arg=“aarch64"
      fi
   fi
   if [ “$input_arg” == “isaac_sim_4.2.0" ]; then
      echo “Building Isaac Sim headless docker”
      dockerfile=“isaac_sim.dockerfile”
      image_tag=“isaac_sim_4.2.0”
      isaac_sim_version=“4.2.0"
   elif [ “$input_arg” == “x86" ]; then
      echo “Building for X86 Architecture”
      dockerfile=“x86.dockerfile”
      image_tag=“x86”
   elif [ “$input_arg” = “aarch64” ]; then
      echo “Building for ARM Architecture”
      dockerfile=“aarch64.dockerfile”
      image_tag=“aarch64"
   else
      echo “Unknown Argument. Please pass one of [x86, aarch64, isaac_sim_2022.2.1, isaac_sim_2023.1.0]”
      exit
   fi
   # build docker file:
   # Make sure you enable nvidia runtime by:
   # Edit/create the /etc/docker/daemon.json with content:
   # {
   #    “runtimes”: {
   #        “nvidia”: {
   #            “path”: “/usr/bin/nvidia-container-runtime”,
   #            “runtimeArgs”: []
   #         }
   #    },
   #    “default-runtime”: “nvidia” # ADD this line (the above lines will already exist in your json file)
   # }
   #
   echo “${dockerfile}”
   docker build --build-arg ISAAC_SIM_VERSION=${isaac_sim_version} -t curobo_docker:${image_tag} -f ${dockerfile} .
   ```
2. **Build Nvidia Docker Container for Isaac Sim and Curobo**
   Run below command to build docker container.
   ```
   bash build_docker.sh isaac_sim_4.2.0
   ```
   You could change isaac_sim_4.2.0 to any tag for your curobo image and later you need to put ```curobo_docker:{custom_image_tag}``` for running your container.
3. **Run docker image**
   Run following command to enable GPUs and network access for docker image.
   ```
   docker run -d --gpus all --network=host \
      -e “PRIVACY_CONSENT=Y” \
      -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
      -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
      -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
      -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
      -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
      -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
      -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
      -v ~/docker/isaac-sim/documents:/root/Documents:rw \
      curobo_docker:isaac_sim_4.2.0
   ```
   This will run the docker image in detached mode and you need to attach to it after the command.
4. **Run Isaac Sim Simulation**
   After getting into the container, get into desired directory.
   ```
   cd curobo
   ```
   Build and compile the repo
   In python script, make sure to enable webrtc app.
   ```
   from omni.isaac.kit import SimulationApp
   simulation_app = SimulationApp({
      “width”: 1280,
      “height”: 720,
      “window_width”: 1920,
      “window_height”: 1080,
      “headless”: True,
      “hide_ui”: False,  # Show the GUI
      “renderer”: “RayTracedLighting”,
      “display_options”: 3286,  # Set display options to show default grid
   })
   simulation_app.set_setting(“/app/window/drawMouse”, True)
   simulation_app.set_setting(“/app/livestream/proto”, “ws”)
   simulation_app.set_setting(“/ngx/enabled”, False)
   from omni.isaac.core.utils.extensions import enable_extension
   enable_extension(“omni.services.streamclient.webrtc”)
   ```
   Once scripts start running, you could access Isaac Sim headless display from same URL. Also make sure there is only one user for headless rendering.
## Troubleshooting
If you encounter any issues:
- Ensure all prerequisites are met
- Check that the server’s firewall allows connections on port 8211
- Verify that the NVIDIA GPU is properly set up and recognized by Docker
For more detailed information, refer to the official NVIDIA Isaac Sim documentation.\
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#container-deployment

[^1]: Last update: 10/17/2024, by Feiyang Ma 
