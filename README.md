# Setting Up PX4 with Ubuntu-Rpi Companion Computer Properly

# 2024 Version
Setup instructions and troubleshooting info to set up Ubuntu 22.04.4 LTS (Jammy Jellyfish) on Rasbperry Pi for ROS2 (Humble) and ensure it works with PX4 stack [v1.14.1](https://github.com/PX4/PX4-Autopilot/tags) via Telem2 Port on Pixhawk 6X Board.

## TODO:
1. Instructions/Troubleshooting guide for configuring radio transmitter and receiver with matching firmware and pairing them
2. Instructions/Troubleshooting for setting the pixhawk paramters on your own to recreate the .params file in this repo

# Setting Up Hardware: Flight Controller and Raspberry Pi Instructions
## Overview of the Setup
0. Set up the pixhawk board for communication with these [instructions](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_rpi.html#ros-2-and-uxrce-dds) (I will make a separate instruciton file to set up the pixhawk board properly for the Holybro X500 V2 using proper QGroundCOntrol Version, flight settings, and Radio Setup)
1. If using Rpi, install Ubuntu on it following these instructions [instructions](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_rpi.html#ubuntu-setup-on-rpi) and [these](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview)
3. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
4. Install ROS2 and MicroRTPS Agent on Rpi to connect to topics from Pixhawk Board using [these](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html#ros-setup-on-rpi) instructions
5. Set up wiring cables using these [instructions](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_rpi.html#wiring)
6. Follow these [instructions](https://docs.px4.io/v1.14/en/ros/ros2_comm.html#build-ros-2-workspace) to set up a ROS2 workspace that contains PX4 message definitions as well as some example code for working with PX4 stack and run the [example](https://docs.px4.io/v1.14/en/ros/ros2_comm.html#running-the-example) to make sure it all works.
     i. ***But make sure you heed the warnings from the section right below on ensuring compatibility with these px4 message definitions and the version of the autopilot running on your pixhawk board***.
8. **Warning**: Compiling px4_msgs takes _FOREVERRRRR_

## Ensuring Compatibility Between Autopilot Software on the Pixhawk Board and Message Definitions on Rpi
In short, ensure that px4_msgs and the PX4 autopilot versions you run are compatible and make sure their ROS_DOMAIN_ID are compatible.

The way that Pixhawk manage their versions was very unintuitive (at least to me), and I'm not sure if other major projects do it the same way.\
As I understand [from here](https://discuss.px4.io/t/ros2-uxrce-agent-cant-subscribe-to-published-topics/35734/8), developmental changes on PX4 repos get pushed on the main branch before they're truly done and compatible with changes on other important repositories. This means that if you follow their [tutorials](https://docs.px4.io/main/en/) and simply git clone whatever repository you need, you may get a version of it that is not only incomplete internally, but also not compatible with other pieces of their code that they tell you to also simply git clone. And when you look at their guide and select for which version you want (main, v1.14, v1.14, etc) it doesn't change which branch/version you should git clone, leaving you lost down the line when either nothing works, or specific important things refuse to work.

This is why you MUST clone specific releases to avoid issues. This information would have saved me MONTHS as a 1st year PhD student (I also didn't really understand git, much less github at the time :P), thinking I had simply made a mistake on my Ubuntu dual-boot that made it useless (trying to get PX4 working was the first thing I always did on a fresh Ubuntu install) and reinstalling Ubuntu and repartitioning my hard drive too many times to count.

### On the Pixhawk Board Autipilot Firmware Side
1. On a _Desktop_ computer (you can't connect pixhawk board to laptop because not enough power goes through). As of June2024 the best stable version is the tagged 1.14.1 release (not the v1.14 release branch). So go to a bash shell as instructed [here](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets) and call: (note: cloning this takes quite a bit)

Clone the PX4-Autopilot Repo
```
https://github.com/PX4/PX4-Autopilot.git
```
You can see all the taggged releases by going to root of PX4-Autopilot and:
```
git tag
```
We want to choose 1.14.1 tagged release
```
git checkout tags/v1.14.1
```
Install Dependencies with their installation tool:
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh #just a shell script that installs dependencies for px4/gazebo/jmavsim/etc
```

1a. **This step is crucial if you intend to connect a radio transmitter to a receiver antenna onboard the drone connected to the pixhawk board to arm/disarm/set to offboard mode/etc**
Go to ~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml and under the very last topic under the Publications umbrella (before it says "subscriptions:", add the following line:
```
  - topic: /fmu/out/rc_channels
    type: px4_msgs::msg::RcChannels
```

It's also nice to get battery information by doing this:
```
  - topic: /fmu/out/battery_status
    type: px4_msgs::msg::BatteryStatus
```
What this does is allow the bridge between uOrb and ROS2 to gain access to the uOrb rc channel topic and message definitions so that they can be used in ROS2. It is not included by default, so you must add this information for the bridge on your own so it connects to the pre-existing uOrb topic/msg definitions.

You can see specifically from [this](https://discuss.px4.io/t/one-ros-topic-disapear/32041/6) response to [this](https://discuss.px4.io/t/one-ros-topic-disapear/32041/6) question, that this topic won't actually appear until and unless the board receives rc_channel inputs from the receiver connected to it because the DDS bridge between uOrb and ROS2 won't generate them if they're unnecessary.

This [page](https://docs.px4.io/main/en/middleware/uxrce_dds.html) of the documentation is very helpful in understanding all of this, especially [this](https://docs.px4.io/main/en/middleware/uxrce_dds.html#supported-uorb-messages) section and [this](https://docs.px4.io/main/en/middleware/uxrce_dds.html#dds-topics-yaml) section

2. Then follow the instructions [here](https://docs.px4.io/main/en/dev_setup/building_px4.html) and specificallly [here](https://docs.px4.io/main/en/dev_setup/building_px4.html#nuttx-pixhawk-based-boards) to build the firmware. It will be in the ~/PX4-Autopilot/build folder of your computer after it's done building. (I'm using a Pixhawk 6x board, so this command below will change depending on specific board)
```
cd ~/PX4-Autopilot
make px4_fmu-v6x_default
```


3. Then you can load it onto the Pixhawk board using the instructions [here](https://docs.px4.io/main/en/config/firmware.html#loading-firmware). Basically open QGroundControl (QGC) on a desktop, click on the Q in the top left, then click Vehicle Setup, then firmware, then connect the Pixhawk board. Now it should pop up with some options that say "PX4 Pro"/"ArduPilot" or a bit further down an option that says "Advanced Settings". Click Advanced Settings and then select "Custom firmware file" from the dropdown menu that appears after you select advanced settings and click ok at the top of the firmware toolbar. Then you do a custom flash where you'll upload the .px4 file in the ~/PX4-Autopilot/build/px4_fmu-v6x_default folder of your computer)

4. You can then go to the parameters section of QGC and click "tools" and then "Load from file" do an upload of the **evannsdoneparams.params** file in this repository to borrow all of my parameters that set up this board specifically for a Holybro x500v2's dimensions, size, motors, etc. It also set up the radio input to interpret each channel on my transmitter how I like it (offboard switch, kill switch, land switch, throttle, angles, etc). Moreover (and probably most importantly), it sets up uxrce_dds to work with ROS2 via the Telem2 port according to the instructions shown above. **It specifically sets the _UXRCE_DDS_DOM_ID_ to 31 to match my ROS_DOMAIN_ID on my computer bash shell and what I will set up on my Rpi**

I have:\
_UXRCE_DDS_DOM_ID_ = 31\
_UXRCE_DDS_PRT_ = 8888\
_UXRCE_DDS_CFG_ = TELEM 2\
_UXRCE_DDS_AG_IP_ = 2130706433 //THIS IS AN IP ADDRESS IN INT32 FORMAT: EX) 192.168.1.13 -> -1062731518 AND 127.0.0.1 -> 2130706433 (I KEPT THE DEFAULT VALUE)\

### Printing uOrb Topics in MAVLink Console
1. List all uOrb topics with:
```
uorb status
```
or
```
ls /obj
```

2. Echo a specific topic with
```
listener [topic_name]
```
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

## Setting Up Static IP Address on Rpi
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

## Making Your Life Easier With Macros
I also recommend these ones to make ssh-ing into the rpis faster
```
alias dronepi='sshpass -p "<PASSWORD>" ssh -t <RPI_NAME>@192.168.X.XXX' #requires sudo apt-get install sshpass
```
And I recommend macros to get udp running fast (there's a way to do this with the password automaticlly piped in but it wouldn't work an all of my Rpi's and I'm not sure why)
```
alias udp='sudo MicroXRCEAgent serial --dev /dev/serial0 -b 921600'
```

# General Tips for Running PX4/Gazebo/ROS2 and Installing Everything You Need For A Smooth Experience
## Conda Instructions
### For Desktop/Laptop
1. Go Follow Instructions [here](https://docs.anaconda.com/miniconda/#quick-command-line-install)
```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```
and then:
```
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
conda config --set auto_activate_base false
```
### For Rpi
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
### Conda Environment from Yaml
From instructions [here](https://shandou.medium.com/export-and-create-conda-environment-with-yml-5de619fe5a2)
1. To save environment as yaml file. Have you environment activated
```
conda env export > environment.yml
```
It will appear in whatever folder you're in
2. To create an environment from yaml file:
```
conda env create -f environment.yml
```
#### yaml tips
1. You probably want to delete the pip dependencies portion of the yml file and only install the conda dependencies. Then take the pip dependencies and put them in this form. Doing it from yml takes forever for some reason
This is the desktop example. wardiNN_desklap_env.yml has these pip dependencies removed. wardiNN_desktop_laptop_env.yaml is the pure yaml as generated orginally.
```
pip install bagpy==0.5 \
            bitstring==4.0.2 \
            casadi==3.6.4 \
            catkin-pkg==0.5.2 \
            cmake==3.27.7 \
            contourpy==1.1.0 \
            cycler==0.11.0 \
            cython==3.0.8 \
            distro==1.8.0 \
            docutils==0.20.1 \
            exceptiongroup==1.1.2 \
            filelock==3.9.0 \
            fonttools==4.40.0 \
            fsspec==2023.4.0 \
            functorch==2.0.0 \
            future-fstrings==1.2.0 \
            gnupg==2.3.1 \
            iniconfig==2.0.0 \
            jinja2==3.0.3 \
            kiwisolver==1.4.4 \
            lit==17.0.6 \
            matplotlib==3.7.2 \
            networkx==3.0 \
            nvidia-cublas-cu11==11.10.3.66 \
            nvidia-cuda-cupti-cu11==11.7.101 \
            nvidia-cuda-nvrtc-cu11==11.7.99 \
            nvidia-cuda-runtime-cu11==11.7.99 \
            nvidia-cudnn-cu11==8.5.0.96 \
            nvidia-cufft-cu11==10.9.0.58 \
            nvidia-curand-cu11==10.2.10.91 \
            nvidia-cusolver-cu11==11.4.0.1 \
            nvidia-cusparse-cu11==11.7.4.91 \
            nvidia-nccl-cu11==2.14.3 \
            nvidia-nvtx-cu11==11.7.91 \
            pillow==10.0.0 \
            pluggy==1.2.0 \
            py3rosmsgs==1.18.2 \
            pycryptodomex==3.18.0 \
            pyparsing==3.0.9 \
            pytest==7.4.0 \
            pyyaml==6.0 \
            rospkg==1.5.0 \
            rowan==1.3.0.post1 \
            seaborn==0.12.2 \
            setuptools-scm==7.1.0 \
            torch==2.0.1 \
            torchaudio==2.1.1+cpu \
            torchdeq==0.1.0 \
            torchvision==0.16.1+cpu \
            triton==2.0.0 \
```

### Torch Installation Tips
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

### Making sure numpy and scikit are compatible:
1. Do [this](https://stackoverflow.com/questions/40845304/runtimewarning-numpy-dtype-size-changed-may-indicate-binary-incompatibility):
```
pip3 uninstall -y numpy scipy pandas scikit-learn
sudo apt update
sudo apt install python3-numpy python3-scipy python3-pandas python3-sklearn
```

### Get tf-transforms:
1. Do [this](https://answers.ros.org/question/384871/how-do-you-install-tf_transformations/):
```
sudo pip3 install transforms3d
sudo apt install ros-<DISTRO>-tf-transformations
```

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
Don't do something like "~/acados/lib". IT WILL NOT WORK. I wasted a week trying to figure out what I had done wrong initially when I made this mistake :).

Straight up just go to your ~/acados/ folder and enter "pwd" command and copy and paste it into where it says "<acados_root>" in step 4 of the python interface installation instructions. Bada bing bada boom.

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

### Fix Deprecated Setuptools Warning
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

   
##  Getting Gazebo Working on a New Desktop/Laptop (esp Ubuntu 22.04+)
#### If you get the error: "ninja: error: unknown target 'gazebo-classic'"
```
sudo apt remove gz-garden
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
```
#### Adding macros to make running iris quad sim and microRTPS easier:
Go to your ~/.bashrc and add the following two lines:
1. Desktop/Laptop-
```
alias udp='MicroXRCEAgent udp4 -p 8888'
alias iris='cd ~/PX4-Autopilot/ && make px4_sitl gazebo-classic'
```

#### Making Gazebo Better (especially on ubuntu 22.04 w/ ros2 galactic)
1. Fix Spawn Point to Be (0,0,0) like [here](https://discuss.px4.io/t/align-px4-local-position-and-gazebo-classic-reference-frame/34038/2)
```
cd /home/evannsm/PX4-Autopilot/Tools/simulation/gazebo-classic
code .
```
Open sitl_run.sh in VSCode. Call "ctrl-G" and go to line 130. Where it should say:
```
while gz model --verbose --spawn-file="${modelpath}/${model}/${model_name}.sdf" --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
		echo "gzserver not ready yet, trying again!"
		sleep 1
	done
```
Here you can update "-x 1.01 -y 0.98 -z 0.83" to:
```
-x 0.0 -y 0.0 -z 0.0
```
2. Make the default world much prettier/closer to wireframe view

```
cd /home/evannsm/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world
```
Open empty.world in VSCode. Go to the about lines 12-14 and comment them out/delete them. These lines:
```
    <!-- <include>
      <uri>model://asphalt_plane</uri>
    </include> -->
```

## Help fully deleting ROS2 from your sysem:
1. https://answers.ros.org/question/57213/how-i-completely-remove-all-ros-from-my-system/
2. Don't forget to delete ros folders @  /etc/ros/

## Other Possibly Helpful Links
1. [PX4-ROS2 Interface Lib](https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html)
2. Fixing topics that don't work randomly after you've built a new px4_msgs and you don't know why it's [not working](https://discuss.px4.io/t/ros2-uxrce-agent-cant-subscribe-to-published-topics/35734/10)
