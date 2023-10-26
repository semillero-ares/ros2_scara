# Ros2 Installation for Ubuntu 

Here is a condensed version of the [official tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS2 on Ubuntu. We must follow a series of commands in the terminal:

## Setup Sources

* Verify the `locale` using: 

`locale`

As a result, `UTF-8` is obtained in all cases.

    LANG=en_US.UTF-8
    LANGUAGE=
    LC_CTYPE="en_US.UTF-8"
    LC_NUMERIC=es_CO.UTF-8
    LC_TIME=es_CO.UTF-8
    LC_COLLATE="en_US.UTF-8"
    LC_MONETARY=es_CO.UTF-8
    LC_MESSAGES="en_US.UTF-8"
    LC_PAPER=es_CO.UTF-8
    LC_NAME=es_CO.UTF-8
    LC_ADDRESS=es_CO.UTF-8
    LC_TELEPHONE=es_CO.UTF-8
    LC_MEASUREMENT=es_CO.UTF-8
    LC_IDENTIFICATION=es_CO.UTF-8
    LC_ALL=

Otherwise we must use:

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

* We will activate the `universe` repository:

        sudo apt install software-properties-common
        sudo add-apt-repository universe

* We install git and other packages:

        sudo apt install git
        sudo apt install libserial-dev

* We will add the security key for the ROS2 repository:

        sudo apt update && sudo apt install curl -y

        sudo curl -ssl https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

* Add the ROS2 repository to the repository list:

        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install ROS 2 packages

* We update the list and update the installed software:

        sudo apt update && sudo apt upgrade

* We install ROS2 humble version:

        sudo apt install ros-humble-desktop

* We install ROS2 packages that we will use:

        sudo apt install ros-humble-gazebo-ros-pkgs

        sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control

        sudo apt install ros-humble-moveit ros-humble-moveit-resources -y


* Install Colcon:

        sudo apt install python3-colcon-common-extensions -y

## Environment setup

* Finally, we configure the terminal to identify the ROS2 commands:

        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

## Verify the installation

* We restart the terminal and then we can test that we have ROS2 correctly installed and running:

    *  In terminal 1:

            source /opt/ros/humble/setup.bash
            ros2 run demo_nodes_cpp talker

    *  In terminal 2:

            source /opt/ros/humble/setup.bash
            ros2 run demo_nodes_py listener

## Troubleshooting

When running ROS2 commands that open a graphical interface like the following:

    ros2 run rviz2 rviz2

An error may take place:

    /opt/ros/humble/lib/rviz2/rviz2: error while loading shared libraries: libQt5Core.so.5: cannot open shared object file: No such file or directory
    [ros2run]: Process exited with failure 127

to solve it we use the following command:

    sudo strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
    


