# Setting Up Ubuntu-Rpi Properly
Troubleshooting info to set up Ubuntu 22.04.4 LTS (Jammy Jellyfish) on Rasbperry Pi for ROS2 (Humble) and ensure it works with PX4 stack via Telem2 Port on Pixhawk 6X Board.

# 2024 Version
## Preliminary Setup
0. Set up the pixhawk board for communication with these [instructions](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_rpi.html#ros-2-and-uxrce-dds) (I will make a separate instruciton file to set up the pixhawk board properly for the Holybro X500 V2 using proper QGroundCOntrol Version, flight settings, and Radio Setup)
1. If using Rpi, install Ubuntu on it using these [instructions](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_rpi.html#ubuntu-setup-on-rpi).
2. Then get it prepared for connection to Pixhawk Board [here](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview)
3. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
4. Install the MicroRTPS Agent to connect to topics from Pixhawk Board
5. Set up wiring cables using these [instructions](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_rpi.html#wiring)
6. Follow these [instructions](https://docs.px4.io/v1.14/en/ros/ros2_comm.html#build-ros-2-workspace) to set up a ROS2 workspace that contains PX4 message definitions as well as some example code for working with PX4 stack and run the [example](https://docs.px4.io/v1.14/en/ros/ros2_comm.html#running-the-example) to make sure it all works.
7. **Warning**: Compiling px4_msgs takes _FOREVERRRRR_

   
## Troubleshooting for Running Everything Smoothly with ROS and PX4
1. Get the package you want to run from your github and put it in your [ros_ws_name]/src/ and then go back to root

### Conda Instructions
You're going to want to use Conda to keep your dependencies in order.
Specifically, use conda-forge's **miniforge** version of conda, as it emphasises supporting various CPU architectures like _aarch64_ which is what the Raspberry Pi 4 Model B uses (you can check your CPU architecture using the "uname -m" command in bash shell). Normal conda/miniconda doesn't seem to be compatible with Rpi in general (to the best of my knowledge). [Conda-forge](https://conda-forge.org/docs/) is community-driven and separate from Anaconda, INC.
1. Install miniforge from the instructions [here](https://github.com/conda-forge/miniforge?tab=readme-ov-file#install) (I like the curl instructions).
2. Once Conda is installed, you can make sure conda commands are [recognized by bash shell using this](https://askubuntu.com/questions/849470/how-do-i-activate-a-conda-environment-in-my-bashrc)
```
echo ". /home/<user>/miniconda3/etc/profile.d/conda.sh" >> ~/.bashrc
```
3. And then prevent base conda environment from opening by default by having conda activated [and then calling this](https://stackoverflow.com/questions/54429210/how-do-i-prevent-conda-from-activating-the-base-environment-by-default)
```
conda config --set auto_activate_base false
```

### Torch Installation Tips
For some reason pytorch has issues installing and then showing up when you run on the shell as opposed to running in a notebook on Vscode. [Best way to install torch](https://stackoverflow.com/questions/54843067/no-module-named-torch) (and also [numpy](https://numpy.org/install/) hehe) is to do:
```
conda create -n **NAME**
conda activate **NAME**
conda config --env --add channels conda-forge
conda install numpy
pip install torchvision
```

### Fix Deprecated Setuptools Warning
When calling colon build in ROS2 you will often see it compile everything perfectly, but throw you a light warning regarding the [depracation regarding setuptools and your setup.cfg file](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)
1. Make sure you have underscores ("_") in setup.cfg file for all of the packages you're calling colcon build on, and not hyphens ("-"). (Should already be like this but could be worth checking)
2. Call this in bash shell according to [instructions](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)
```
pip install setuptools==58.2.0
```

### Now you should be able to build everything with no errors!
1. On a clean bash shell to ROS2 workspace with px4_msgs and px4_ros_com already compiled
2. Make sure your packages are in the ros2ws/src/ folder ready to be built
3. Make sure conda is NOT activated
4. Make sure workspace is NOT sourced
5. Call this on the package you want to build:
```
colcon build --packages-select **PACKAGE_NAME** --symlink-install
```
6. The **symlink-install** ensures that you can edit python files and not have to rebuild the package every time!
