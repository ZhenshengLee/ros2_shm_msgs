#!/usr/bin/env bash

# 进入某个容器
# 必须制定一个参数：容器名
# 选择制定一个参数：是否root，默认非root

GA_DIST=""

function show_usage()
{
cat <<EOF
Usage: $(basename $0) [options] ...
OPTIONS:
    -d 22.04
    -d 20.04
EOF
exit 0
}

# 参数校验
while [ $# -gt 0 ]
do
    case "$1" in
    -d|--dist)
        VAR=$1
        GA_DIST=$VAR
        ;;
    -h|--help)
        show_usage
        ;;
    stop)
    stop_containers
    exit 0
    ;;
    *)
        echo -e "\033[93mWarning\033[0m: Unknown option: $1"
        exit 2
        ;;
    esac
    shift
done

# zs:
# 如果没有制定,默认是22.04
if [ -z "${GA_DIST}" ]; then
    GA_DIST=22.04
else
    GA_DIST=20.04
fi

xhost +local:root 1>/dev/null 2>&1
docker exec \
    -u $USER \
    -e HISTFILE=/ga_ros/.dev_bash_hist \
    -it zs_${GA_DIST}_$USER \
    /bin/bash

# docker exec \
#     -u root \
#     -e HISTFILE=/ga_ros/.dev_bash_hist \
#     -it zs_${GA_DIST}_$USER \
#     /bin/bash

xhost -local:root 1>/dev/null 2>&1