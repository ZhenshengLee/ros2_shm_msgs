#!/usr/bin/env bash

# 添加用户和组权限，配置超级用户

addgroup --gid "$DOCKER_GRP_ID" "$DOCKER_GRP"
adduser --disabled-password --force-badname --gecos '' "$DOCKER_USER" \
    --uid "$DOCKER_USER_ID" --gid "$DOCKER_GRP_ID" 2>/dev/null
usermod -aG sudo "$DOCKER_USER"
echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# 删除一些目录
# rm -rf build/ devel/

# 更换国内源
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
sudo sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/' /etc/apt/sources.list
sudo cp /ga_ros/ga_deps/ros2_shm_msgs/scripts/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
# sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo sed -i 's/packages.ros.org/mirrors.tuna.tsinghua.edu.cn/' /etc/apt/sources.list.d/ros2.list


# 检测是16.04还是18.04

# case "$(lsb_release -r | cut -f2)" in
#     16.04)
#         echo '
#         source /opt/ros/kinetic/setup.bash
#         ' >> "/home/${DOCKER_USER}/.bashrc"
#         ;;
#     *)
#         echo '
#         source /opt/ros/melodic/setup.bash
#         ' >> "/home/${DOCKER_USER}/.bashrc"
#         ;;
# esac

# 设置bashrc
cat /ga_ros/ga_deps/ros2_shm_msgs/scripts/.bashrc > "/home/${DOCKER_USER}/.bashrc"

# echo '
# genhtml_branch_coverage = 1
# lcov_branch_coverage = 1
# ' > "/home/${DOCKER_USER}/.lcovrc"

# Set user files ownership to current user, such as .bashrc, .profile, etc.
chown ${DOCKER_USER}:${DOCKER_GRP} /home/${DOCKER_USER}
ls -ad /home/${DOCKER_USER}/.??* | xargs chown -R ${DOCKER_USER}:${DOCKER_GRP}

# 配置一些设备最基本的属性

# setup GPS device
if [ -e /dev/novatel0 ]; then
  chmod a+rw /dev/novatel0
fi
if [ -e /dev/novatel1 ]; then
  chmod a+rw /dev/novatel1
fi
if [ -e /dev/novatel2 ]; then
  chmod a+rw /dev/novatel2
fi
if [ -e /dev/ttyACM0 ]; then
  chmod a+rw /dev/ttyACM0
fi

# setup camera device
if [ -e /dev/camera/obstacle ]; then
  chmod a+rw /dev/camera/obstacle
fi
if [ -e /dev/camera/trafficlights ]; then
  chmod a+rw /dev/camera/trafficlights
fi
