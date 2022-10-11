# docker

## install

### install docker-ce


```sh
sudo apt install apt-transport-https ca-certificates curl software-properties-common
# curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
# sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"
curl -fsSL https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu $(lsb_release -cs) stable"
apt-cache policy docker-ce
sudo apt install docker-ce docker-compose
sudo systemctl status docker
# 配置
如果您想在运行docker命令时避免键入sudo ，请将您的用户名添加到docker组：
sudo groupadd docker
sudo usermod -aG docker ${USER}
要应用新的组成员身份，请注销服务器并重新登录，或键入以下内容：
su - ${USER}
 系统将提示您输入用户密码以继续。
通过键入以下内容确认您的用户现已添加到docker组：
id -nG
如果您需要将用户添加到您未登录的docker组，请使用以下命令明确声明该用户名：
sudo usermod -aG docker zs
# 使用
docker [option] [command] [arguments]
# 查看所有命令
docker

sudo dockerd --debug
```

### source

`/etc/docker/daemon.json`

```json
{
    "registry-mirrors": [
        "http://hub-mirror.c.163.com"
    ]
}
```

修改Docker配置文件/etc/default/docker如下：
DOCKER_OPTS="--registry-mirror=http://hub-mirror.c.163.com"

```sh
sudo service docker restart
docker info
docker pull ubuntu:22.04
docker run -itd --name ubuntu-test ubuntu:22.04
docker exec -it ubuntu-test /bin/bash
```

### 安装nvidia-container-toolkit

```sh
# 添加源
$ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
$ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
$ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
# 安装并重启docker
$ sudo apt update && sudo apt install -y nvidia-container-toolkit
$ sudo systemctl restart docker
```

## development

```sh
docker import - ga_team/ga_all:18.04 < ga_all_1804.tar
./ga_all_start.sh -l
./ga_all_into.sh
```

### 保存新镜像

```sh
docker export 50d941249065 > ga_all_1804.tar
```

### extra command

```sh
docker save 9100b0f32f3f > melodic-perception-bionic.tar
# 保存多个镜像为一个文件
docker save -o images.tar postgres:9.6 mongo:3.4
# 加载文件到镜像
docker load < melodic-perception-bionic.tar
docker load -i xos.tar

docker export aad32d47cbcb > od_ros_1804.tar
docker import - od_team/od_ros:18.04 < od_ros_1804.tar
# 删除image
docker image rm d9d4eff81e69

```

## concepts

### docker save/load vs docker export/import

```sh
# docker save will indeed produce a tarball, but with all parent layers, and all tags + versions.
# docker export does also produce a tarball, but without any layer/history.

# https://www.baeldung.com/ops/docker-save-export
# The Docker save command is used to save a Docker image to a tar file.
# The Docker export command is used to save a Docker container to a tar file.

```

### docker export vs docker commit

https://www.cnblogs.com/JetpropelledSnake/p/10518004.html

```sh
# commit是合并了save、load、export、import这几个特性的一个综合性的命令，它主要做了：

# 将container当前的读写层保存下来，保存成一个新层
# 和镜像的历史层一起合并成一个新的镜像
# 如果原本的镜像有3层，commit之后就会有4层，最新的一层为从镜像运行到commit之间对文件系统的修改
```

### tips when create images

https://www.dataset.com/blog/create-docker-image/

```sh
# check layers
docker inspect ubuntu:22.04

# befor docker export
rm -rf /var/lib/apt/lists/*
```

### layers

读写层和历史层

### commit with msg

此外，使用 docker commit 意味着所有对镜像的操作都是黑箱操作，生成的镜像也被称为 黑箱镜像，换句话说，就是除了制作镜像的人知道执行过什么命令、怎么生成的镜像，别人根本无从得知。而且，即使是这个制作镜像的人，过一段时间后也无法记清具体的操作。这种黑箱镜像的维护工作是非常痛苦的。

```sh
docker commit -m

docker commit \
    --author "Tao Wang <twang2218@gmail.com>" \
    --message "修改了默认网页" \
    webserver \
    nginx:v2

docker diff
docker history
```

### layers merge

https://stackoverflow.com/questions/56117261/how-to-merge-dockers-layers-of-image-and-slim-down-the-image-file

https://medium.com/@gdiener/how-to-build-a-smaller-docker-image-76779e18d48a

## using dockerfile

rather than docker commit/export

todo
