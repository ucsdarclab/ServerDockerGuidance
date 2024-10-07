# Run Isaac Sim Remotely ARCLAB using webrtc

This guide provides step-by-step instructions for running NVIDIA's Isaac Sim remotely from the lab environment.

## Prerequisites
 
- UCSD Wifi or VPN
- NGC account for accessing NVIDIA containers (opitonal if docker already pulled)

## Steps

1. **SSH into the remote server**
   ```
   ssh username@blackdragon1.ucsd.edu
   ```
   Enter your password when prompted.

2. **Check the server's IP address**
   ```
   hostname -I | awk '{print $1}'
   ```
   Note this IP address for later use.

3. **Generate NGC API Key**
   - Log into your NGC account and generate an API key.
   - https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#container-deployment

4. **Log in to the NVIDIA Container Registry**
   ```
   docker login nvcr.io
   ```
   - Username: `$oauthtoken`
   - Password: Your NGC API Key

5. **Pull the Isaac Sim Docker image**
   ```
   docker pull nvcr.io/nvidia/isaac-sim:4.2.0
   ```

6. **Run the Isaac Sim Docker container**
   ```
   docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
   -e "PRIVACY_CONSENT=Y" \
   -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
   -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
   -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
   -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
   -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
   -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
   -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
   -v ~/docker/isaac-sim/documents:/root/Documents:rw \
   nvcr.io/nvidia/isaac-sim:4.2.0
   ```

7. **Start Isaac Sim in headless WebRTC mode**
   In the new Docker bash session, run:
   ```
   ./isaac-sim.headless.webrtc.sh --allow-root
   ```

8. **Access Isaac Sim remotely**
   On your local machine, open a web browser and navigate to:
   ```
   http://<server_ip>:8211/streaming/webrtc-demo/?server=<server_ip>
   ```
   Replace `<server_ip>` with the IP address you noted in step 2. 
   
   Note: 8211 is the default Isaac Sim port.


9. **Run standalone program remotely**
    
    Following is an example of running standalone program remotely with webrtc

    ```
    from isaacsim import SimulationApp

    # This sample enables a livestream server to connect to when running headless
    CONFIG = {
        "width": 1280,
        "height": 720,
        "window_width": 1920,
        "window_height": 1080,
        "headless": True,
        "hide_ui": False,  # Show the GUI
        "renderer": "RayTracedLighting",
        "display_options": 3286,  # Set display options to show default grid
    }


    # Start the omniverse application
    kit = SimulationApp(launch_config=CONFIG)

    from omni.isaac.core.utils.extensions import enable_extension

    # Default Livestream settings
    kit.set_setting("/app/window/drawMouse", True)
    kit.set_setting("/app/livestream/proto", "ws")
    kit.set_setting("/ngx/enabled", False)

    # Note: Only one livestream extension can be enabled at a time

    # Enable Native Livestream extension
    # Default App: Streaming Client from the Omniverse Launcher
    # enable_extension("omni.kit.streamsdk.plugins-3.2.1")
    # enable_extension("omni.kit.livestream.core-3.2.0")
    # enable_extension("omni.kit.livestream.native")

    # Enable WebRTC Livestream extension
    # Default URL: http://localhost:8211/streaming/webrtc-client/
    enable_extension("omni.services.streamclient.webrtc")

    from omni.isaac.core import World
    world = World()
    world.scene.add_default_ground_plane()

    # Run until closed
    while kit._app.is_running() and not kit.is_exiting():
        # Run in realtime mode, we don't specify the step size
        kit.update()

    kit.close()
    ```


## Troubleshooting

If you encounter any issues:
- Ensure all prerequisites are met
- Check that the server's firewall allows connections on port 8211
- Verify that the NVIDIA GPU is properly set up and recognized by Docker

For more detailed information, refer to the official NVIDIA Isaac Sim documentation.\
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#container-deployment