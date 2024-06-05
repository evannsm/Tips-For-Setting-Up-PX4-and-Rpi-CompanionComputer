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

## Compatibility Tips Between Autopilot Software on the Pixhawk Board and Message Definitions
Ensure that px4_msgs and the PX4 autopilot versions you run are compatible and make sure their ROS_DOMAIN_ID are compatible

### On the Pixhawk Board Autipilot Firmware Side
1. On a _Desktop_ computer (you can't connect pixhawk board to laptop because not enough power goes through). As of June2024 the best stable version is release 1.14 for everything. So go to a bash shell as instructed (here)[https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html] and call
```
git clone -b release/1.14 https://github.com/PX4/PX4-Autopilot.git --recursive
```
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

2. Then follow the instructions [here] (https://docs.px4.io/main/en/dev_setup/building_px4.html) to build the firmware. It will be in the ~/PX4-Autopilot/build folder of your computer after it's done building. (I'm using a Pixhawk 6x board, so this command below will change depending on specific board)
```
make px4_fmu-v6x_default
```

3. Then you can load it onto the Pixhawk board using the instructions [here](https://docs.px4.io/main/en/config/firmware.html#installing-px4-main-beta-or-custom-firmware). Basically open QGroundControl (QGC) on a desktop, click on the Q in the top left, then click Vehicle Setup, then firmware, then connect the Pixhawk board, and then do a custom flash where you'll upload the .px4 file in the ~/PX4-Autopilot/build/px4_fmu-v6x_default folder of your computer)

4. You can then go to the parameters section of QGC and do an upload of the **evannsdoneparams.params** file in this repository to borrow all of my parameters that set up this board specifically for a Holybro x500v2's dimensions, size, motors, etc. It also set up the radio input to interpret each channel on my transmitter how I like it (offboard switch, kill switch, land switch, throttle, angles, etc). Moreover (and probably most importantly) it sets up uxrce_dds to work with ROS2 via the Telem2 port according to the instructions shown above. **It specifically sets the ROS_DOMAIN_ID to be 31 to match what I have on my computer and what I will set up on my Rpi**

I have:\
_UXRCE_DDS_DOM_ID_ = 31\
_UXRCE_DDS_PRT_ = 8888\
_UXRCE_DDS_CFG_ = TELEM 2\
_UXRCE_DDS_AG_IP_ = 2130706433 //THIS IS AN IP ADDRESS IN INT32 FORMAT: EX) 192.168.1.13 -> -1062731518 AND 127.0.0.1 -> 2130706433 (I KEPT THE DEFAULT VALUE)\

### On the Raspbery Pi Side
1. When you follow the instructions to set up a ros2 workspace for PX4 work, make sure you do this:
```
mkdir -p ros2_ws/src
git clone -b release/1.14 https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build
```
This ensures that you have the compatible ros2 message definitions for the px4 firmware you have
2. Go to your ~/.bashrc and make sure you have the same ROS_DOMAIN_ID by adding this line to the bottom of your bashrc file
```
export ROS_DOMAIN_ID=31 # To ensure domain matches what is on the Pixhawk board and my computer
```

## Setting up static IP address on Rpi
1. Open a bash shell and get the following pieces of information:
   1. Your Rpi's current IPv4 address: ```hostname -I``` -> yields something like **192.168.1.27**
   2. Your router's IP address AKA "Gateway": ```ip r | grep default``` -> yields something like **default via 192.168.1.1 dev wlan0 proto static metric 20600**
   3. DNS Server: ```grep "nameserver" /etc/resolv.conf``` -> yields something like **nameserver 127.0.0.53**
  
2. Now that you have the necessary information:
   1. Call the following: ```sudo nmtui```
   2. Go to _Edit a connection_
   3. Go to the GRITS_Downstairs option under Wi-Fi and hit enter
   4. Change IPv4 Configuration to <Manual> under its dropdown menu
   5. For Addresses put in your desired Rpi static IP address (you can make it the result under "hostname -I" to ensure it's not already taken but on this particular network you can choose your own value)
   6. Under Gateway put in the router's IP address from "ip r | grep default"
   7. Under DNS servers put in the result after **nameserver** from above
   8. Make sure the "Automatically Connect" setting is checked and then hit OK
   9. Go back and quit the nmtui interface
   10. Reboot with ```sudo reboot```
  
3. Now you can go to your main laptop and make sure you're on the same network and call ```ping RPI_IP_ADDRESS``` and it should return successful
4. now try ```ssh rpi_device_name@RPI_IP_ADDRESS``` HA IT WONT WORK!
5. Now go back to the Rpi and call ```sudo apt install openssh-server```
6. NOW it should work when you call ```ssh rpi_device_name@RPI_IP_ADDRESS```
   
## Troubleshooting for Running Everything Smoothly with ROS and PX4

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

#### Torch Installation Tips
If you need torch for running certain code you may try running a conda env and conda install torch on it and see that when you run conda activate and source install/setup.bash, and then do "ros2 run package script" it wont work!!!!! ROS2 won't see that you have torch installed no matter what. Torch is weird like that with ROS2. It'll run on a ipynb, normal script, and even in the shell but not when running with "ros2 run ..." 

This hopefully will help fix this issue.
For some reason pytorch has issues installing and then showing up when you run on the shell as opposed to running in a notebook on Vscode. [Best way to install torch](https://stackoverflow.com/questions/54843067/no-module-named-torch) (and also [numpy](https://numpy.org/install/) hehe) is to do:
```
conda create -n **NAME**
conda activate **NAME**
conda install numpy
conda install scipy
conda deactivate
pip install torchvision
```
Should work now. Or maybe not. Just keep installing it every way possible until it works :P

## Acados & AcadosPython Interface Installation Tips
This will probably be a pain in the ass. These tips should eliminate most if not all of that pain.

You want to use the [CMake installation](https://docs.acados.org/installation/index.html) for Linux/Mac as instructed in the python interface installation [instructions](https://docs.acados.org/python_interface/index.html)
1. Just note that when it says "Add the path to the compiled shared libraries" by adding the export lines to the .bashrc file, you don't want to use the ~/ shortcut, you want the fully spelled out without the ~/ shortcut to your user's home directory.
Example:
In your .bashrc you want to add:
```
# FOR ACADOS
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/*USERNAME*/acados/lib"
export ACADOS_SOURCE_DIR="/home/*USERNAME*/acados/"
```
Don't do something like "~/acados/lib". IT WILL NOT WORK. I wasted a week trying to figure out what I had done wrong initially :) when I made this mistake.

Straight up just go to your ~/acados/ folder and enter "pwd" command and copy and paste it into where it says "<acados_root>" in step 4 of the python interface installation instructions. Badabim badaboom.

2. Now, the next issue will come from t_renderer. The instructions will ask you to do the following
```
cd ~/acados/examples/acados_python/getting_started
```
```
run python3 minimal example_ocp.py
```
It will then prompt you to download and save t_renderer in ~/acados/bin. This might work for some, it has never worked for me.

The best way to go about this is:
3. Go to [tera renderer github](https://github.com/acados/tera_renderer) and follow their instructions:
You want to clone the directory into your home folder ~/
```
git clone https://github.com/acados/tera_renderer
```

Then make sure you can call the cargo command by using [rustup](https://www.rust-lang.org/tools/install)
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Once this is done you can test your cargo install by simply calling "cargo" in your shell. Once you confirm that cargo is working, go to ~/tera_renderer and call this command:
```
cargo build --verbose --release
```

It should take around 4min on Rpi to compile. Then take the newly created t_renderer (in green bc it's executable) file in ~/tera_renderer/target/releaseand copy it into the ~/acados/bin folder.

Now you can call
```
run python3 minimal example_ocp.py
```

And it should run perfectly.

## Fix Deprecated Setuptools Warning
When calling colon build in ROS2 you will often see it compile everything perfectly, but throw you a light warning regarding the [depracation regarding setuptools and your setup.cfg file](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)
1. Make sure you have underscores ("_") in setup.cfg file for all of the packages you're calling colcon build on, and not hyphens ("-"). (Should already be like this but could be worth checking)
2. Call this in bash shell according to [instructions](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)
```
pip install setuptools==58.2.0
```

### Now you should be able to build everything with no errors or warnings!
1. On a clean bash shell to ROS2 workspace with px4_msgs and px4_ros_com already compiled
2. Make sure your packages are in the ros2ws/src/ folder ready to be built
3. Make sure conda is NOT activated
4. Make sure workspace is NOT sourced
5. Call this on the package you want to build:
```
colcon build --packages-select **PACKAGE_NAME** --symlink-install
```
6. The **symlink-install** ensures that you can edit python files and not have to rebuild the package every time!

## Other Possibly Helpful Links
1. [PX4-ROS2 Interface Lib](https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html)
2. Fixing topics that don't work randomly after you've built a new px4_msgs and you don't know why it's [not working](https://discuss.px4.io/t/ros2-uxrce-agent-cant-subscribe-to-published-topics/35734/10)
