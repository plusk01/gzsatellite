Gazebo Satellite
================

ROS/Gazebo plugin to create a satellite-imagery world based on a GPS location.

## Considerations ##

This plugin allows you to pull in arbitrarily large satellite imagery into Gazebo. Of course, that doesn't mean you should. If you have a GPU, the large model that Gazebo creates could use a large portion of your VRAM for graphics rendering, causing other GPU processes (such as compute processes) to crash. For example, pulling in a 400x400m region at zoom level 22 took ~1GB of VRAM for me. This did not leave enough resources for other GPU compute processes, causing crashes. If you have an NVIDIA GPU, you can check VRAM usage with the `nvidia-smi` command. Combining this command with `watch -n 0.1 nvidia-smi` allows you to watch your GPU resources in real time.