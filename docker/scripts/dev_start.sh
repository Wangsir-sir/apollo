#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# 此脚本的功能：检查主机环境，拉取所需的镜像，运行镜像将其中的数据保存至数据卷当中
# 运行镜像，容器后台运行

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${CURR_DIR}/docker_base.sh"

CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"

DOCKER_REPO="apolloauto/apollo" # 镜像的仓库名称
DEV_CONTAINER="apollo_dev_${USER}"
DEV_INSIDE="in-dev-docker"

SUPPORTED_ARCHS=(x86_64 aarch64)
TARGET_ARCH="$(uname -m)"

# X86_64架构下默认启动的镜像名称
VERSION_X86_64="dev-x86_64-18.04-20210517_1712"
TESTING_VERSION_X86_64="dev-x86_64-18.04-testing-20210112_0008"

VERSION_AARCH64="dev-aarch64-18.04-20201218_0030"
USER_VERSION_OPT= # 手动指定启动的镜像版本

FAST_MODE="no"

GEOLOC= # 镜像仓库网站

USE_LOCAL_IMAGE=0
CUSTOM_DIST= # 启动镜像的发行版本(stable/testing)
USER_AGREED="no"

VOLUME_VERSION="latest"
SHM_SIZE="2G"
USER_SPECIFIED_MAPS=
MAP_VOLUMES_CONF=
OTHER_VOLUMES_CONF=

# 默认要拉取的地图容器名
DEFAULT_MAPS=(
    sunnyvale_big_loop
    sunnyvale_loop
    sunnyvale_with_two_offices
    san_mateo
)

# Fast mode下要拉取的地图容器名
DEFAULT_TEST_MAPS=(
    sunnyvale_big_loop
    sunnyvale_loop
)

function show_usage() {
    cat <<EOF
Usage: $0 [options] ...
OPTIONS:
    -h, --help             Display this help and exit.
    -f, --fast             Fast mode without pulling all map volumes.
    -g, --geo <us|cn|none> Pull docker image from geolocation specific registry mirror.
    -l, --local            Use local docker image.
    -t, --tag <TAG>        Specify docker image with tag <TAG> to start.
    -d, --dist             Specify Apollo distribution(stable/testing)
    --shm-size <bytes>     Size of /dev/shm . Passed directly to "docker run"
    -y                     Agree to Apollo License Agreement non-interactively.
    stop                   Stop all running Apollo containers.
EOF
}

# 解析参数，并根据参数执行相应的语句
function parse_arguments() {
    local custom_version="" # 启动镜像的版本
    local custom_dist="" #启动镜像的发行版本(stable/testing)
    local shm_size=""
    local geo=""

    while [ $# -gt 0 ]; do
        local opt="$1"
        shift
        case "${opt}" in
            -t | --tag)
                # -n字符串长度不为0为真
                if [ -n "${custom_version}" ]; then
                    warning "Multiple option ${opt} specified, only the last one will take effect."
                fi
                custom_version="$1"
                shift
                optarg_check_for_opt "${opt}" "${custom_version}"
                ;;

            -d | --dist)
                custom_dist="$1"
                shift
                optarg_check_for_opt "${opt}" "${custom_dist}"
                ;;

            -h | --help)
                show_usage
                exit 1
                ;;

            -f | --fast)
                FAST_MODE="yes"
                ;;

            -g | --geo)
                geo="$1"
                shift
                optarg_check_for_opt "${opt}" "${geo}"
                ;;

            -l | --local)
                USE_LOCAL_IMAGE=1
                ;;

            --shm-size)
                shm_size="$1"
                shift
                optarg_check_for_opt "${opt}" "${shm_size}"
                ;;

            --map)
                map_name="$1"
                shift
                USER_SPECIFIED_MAPS="${USER_SPECIFIED_MAPS} ${map_name}"
                ;;
            -y)
                USER_AGREED="yes"
                ;;
            stop)
                info "Now, stop all Apollo containers created by ${USER} ..."
                stop_all_apollo_containers "-f"
                exit 0
                ;;
            *)
                warning "Unknown option: ${opt}"
                exit 2
                ;;
        esac
    done # End while loop

    # 如果字符串不为空就会执行后面的语句
    [[ -n "${geo}" ]] && GEOLOC="${geo}"
    [[ -n "${custom_version}" ]] && USER_VERSION_OPT="${custom_version}"
    [[ -n "${custom_dist}" ]] && CUSTOM_DIST="${custom_dist}"
    [[ -n "${shm_size}" ]] && SHM_SIZE="${shm_size}"
}

# 根据版本确定运行的镜像DEV_IMAGE
function determine_dev_image() {
    local version="$1"
    # If no custom version specified
    if [[ -z "${version}" ]]; then
        if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
            if [[ "${CUSTOM_DIST}" == "testing" ]]; then
                version="${TESTING_VERSION_X86_64}"
            else
                version="${VERSION_X86_64}"
            fi
        elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
            version="${VERSION_AARCH64}"
        else
            error "Logic can't reach here! Please report this issue to Apollo@GitHub."
            exit 3
        fi
    fi
    DEV_IMAGE="${DOCKER_REPO}:${version}"
}

# 检查主机系统是否为Linux
function check_host_environment() {
    if [[ "${HOST_OS}" != "Linux" ]]; then
        warning "Running Apollo dev container on ${HOST_OS} is UNTESTED, exiting..."
        exit 1
    fi
}

# 检查主机的架构
function check_target_arch() {
    local arch="${TARGET_ARCH}"
    # @代表数组当中所有的元素
    for ent in "${SUPPORTED_ARCHS[@]}"; do
        if [[ "${ent}" == "${TARGET_ARCH}" ]]; then
            return 0
        fi
    done
    error "Unsupported target architecture: ${TARGET_ARCH}."
    exit 1
}

# 检查设备是否存在并且建立数据卷的参数
function setup_devices_and_mount_local_volumes() {
    local __retval="$1"

    # 若.cache文件夹不存在则创建一个
    [ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

    source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh"
    setup_device

    local volumes="-v $APOLLO_ROOT_DIR:/apollo"
    local teleop="${APOLLO_ROOT_DIR}/../apollo-teleop"
    if [ -d "${teleop}" ]; then
        volumes="-v ${teleop}:/apollo/modules/teleop ${volumes}"
    fi

    local os_release="$(lsb_release -rs)"
    case "${os_release}" in
        16.04)
            warning "[Deprecated] Support for Ubuntu 16.04 will be removed" \
                "in the near future. Please upgrade to ubuntu 18.04+."
            volumes="${volumes} -v /dev:/dev"
            ;;
        18.04 | 20.04 | *)
            volumes="${volumes} -v /dev:/dev"
            ;;
    esac
    # local tegra_dir="/usr/lib/aarch64-linux-gnu/tegra"
    # if [[ "${TARGET_ARCH}" == "aarch64" && -d "${tegra_dir}" ]]; then
    #    volumes="${volumes} -v ${tegra_dir}:${tegra_dir}:ro"
    # fi
    volumes="${volumes} -v /media:/media \
                        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                        -v /etc/localtime:/etc/localtime:ro \
                        -v /usr/src:/usr/src \
                        -v /lib/modules:/lib/modules"
    volumes="$(tr -s " " <<<"${volumes}")"
    # Apollo当中所有的指定数据卷
    # volumes= -v /home/wcx/apollo:/apollo -v /dev:/dev -v /media:/media -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    #          -v /etc/localtime:/etc/localtime:ro -v /usr/src:/usr/src -v /lib/modules:/lib/modules


    eval "${__retval}='${volumes}'"
}

# 根据镜像名拉取镜像
function docker_pull() {
    local img="$1"
    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        # 若本地有要指定拉取的镜像，则返回。 
        # --format会以指定格式输出镜像信息。grep -q没有匹配信息返回0
        if docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "${img}"; then
            info "Local image ${img} found and will be used."
            return
        fi
        warning "Image ${img} not found locally although local mode enabled. Trying to pull from remote registry."
    fi
    # 根据指定的仓库网址对镜像名进行修改
    if [[ -n "${GEO_REGISTRY}" ]]; then
        img="${GEO_REGISTRY}/${img}"
    fi

    # 拉取镜像
    info "Start pulling docker image ${img} ..."
    if ! docker pull "${img}"; then
        error "Failed to pull docker image : ${img}"
        exit 1
    fi
}

# 拉取镜像，运行容器并挂载
# 由于容器没有前台进程，运行后会退出并停止，只是将其中的地图文件保存到数据卷当中
function docker_restart_volume() {
    local volume="$1" # 数据卷的名字
    local image="$2" # 镜像名
    local path="$3" #挂载在容器当中的路径
    info "Create volume ${volume} from image: ${image}"
    docker_pull "${image}"
    docker volume rm "${volume}" >/dev/null 2>&1
    # 将数据卷volume挂载到容器当中的指定路径path，并运行容器
    docker run -v "${volume}":"${path}" --rm "${image}" true
}

# 根据数据卷的名字和数据卷容器的版本，拉取镜像并运行容器，挂载数据卷到容器的指定路径
function restart_map_volume_if_needed() {
    local map_name="$1" # 指定数据卷的名字
    local map_version="$2" # 指定数据卷容器的版本
    local map_volume="apollo_map_volume-${map_name}_${USER}" # 数据卷的名字
    local map_path="/apollo/modules/map/data/${map_name}" # 挂载在容器当中的路径

    if [[ ${MAP_VOLUMES_CONF} == *"${map_volume}"* ]]; then
        info "Map ${map_name} has already been included."
    else
        local map_image=
        if [ "${TARGET_ARCH}" = "aarch64" ]; then
            map_image="${DOCKER_REPO}:map_volume-${map_name}-${TARGET_ARCH}-${map_version}"
        else
            map_image="${DOCKER_REPO}:map_volume-${map_name}-${map_version}"
        fi
        info "Load map ${map_name} from image: ${map_image}"

        docker_restart_volume "${map_volume}" "${map_image}" "${map_path}"
        MAP_VOLUMES_CONF="${MAP_VOLUMES_CONF} --volume ${map_volume}:${map_path}"
    fi
}

# 拉取并运行所有的map容器，并将其中的地图文件保存到数据卷当中
# 每个map容器当中，/apollo/modules/map/data/${map_name}/文件夹下保存了该地图的文件，
# 并建立数据卷，挂载到该容器的此目录，因此每个map容器的地图会保存到数据卷当中
function mount_map_volumes() {
    info "Starting mounting map volumes ..."
    if [ -n "${USER_SPECIFIED_MAPS}" ]; then
        for map_name in ${USER_SPECIFIED_MAPS}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    fi

    if [[ "$FAST_MODE" == "no" ]]; then
        for map_name in ${DEFAULT_MAPS[@]}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    else
        for map_name in ${DEFAULT_TEST_MAPS[@]}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    fi
}

# 拉取并运行所有的需要的容器，并将其中的文件保存至数据卷当中
function mount_other_volumes() {
    info "Mount other volumes ..."
    local volume_conf=

    # AUDIO
    local audio_volume="apollo_audio_volume_${USER}"
    local audio_image="${DOCKER_REPO}:data_volume-audio_model-${TARGET_ARCH}-latest"
    local audio_path="/apollo/modules/audio/data/"
    docker_restart_volume "${audio_volume}" "${audio_image}" "${audio_path}"
    volume_conf="${volume_conf} --volume ${audio_volume}:${audio_path}"

    # YOLOV4
    local yolov4_volume="apollo_yolov4_volume_${USER}"
    local yolov4_image="${DOCKER_REPO}:yolov4_volume-emergency_detection_model-${TARGET_ARCH}-latest"
    local yolov4_path="/apollo/modules/perception/camera/lib/obstacle/detector/yolov4/model/"
    docker_restart_volume "${yolov4_volume}" "${yolov4_image}" "${yolov4_path}"
    volume_conf="${volume_conf} --volume ${yolov4_volume}:${yolov4_path}"

    # FASTER_RCNN
    local faster_rcnn_volume="apollo_faster_rcnn_volume_${USER}"
    local faster_rcnn_image="${DOCKER_REPO}:faster_rcnn_volume-traffic_light_detection_model-${TARGET_ARCH}-latest"
    local faster_rcnn_path="/apollo/modules/perception/production/data/perception/camera/models/traffic_light_detection/faster_rcnn_model"
    docker_restart_volume "${faster_rcnn_volume}" "${faster_rcnn_image}" "${faster_rcnn_path}"
    volume_conf="${volume_conf} --volume ${faster_rcnn_volume}:${faster_rcnn_path}"

    # SMOKE
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        local smoke_volume="apollo_smoke_volume_${USER}"
        local smoke_image="${DOCKER_REPO}:smoke_volume-yolo_obstacle_detection_model-${TARGET_ARCH}-latest"
        local smoke_path="/apollo/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector/smoke_libtorch_model"
        docker_restart_volume "${smoke_volume}" "${smoke_image}" "${smoke_path}"
        volume_conf="${volume_conf} --volume ${smoke_volume}:${smoke_path}"
    fi

    OTHER_VOLUMES_CONF="${volume_conf}"
}

function main() {
    # 检查主机系统是否为Linux
    check_host_environment
    # 检查主机的架构
    check_target_arch

    # 根据输入不同的参数，进行相应的设置
    parse_arguments "$@"

    # 检查用户协议
    if [[ "${USER_AGREED}" != "yes" ]]; then
        check_agreement
    fi

    # 根据版本确定运行的镜像DEV_IMAGE
    determine_dev_image "${USER_VERSION_OPT}"
    # 确定镜像仓库网站GEO_REGISTRY
    geo_specific_config "${GEOLOC}"

    # 拉取镜像，并可选择使用以及拉取的本地镜像
    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        info "Start docker container based on local image : ${DEV_IMAGE}"
    fi

    # 根据镜像名拉取DEV_IMAGE镜像
    if ! docker_pull "${DEV_IMAGE}"; then
        error "Failed to pull docker image ${DEV_IMAGE}"
        exit 1
    fi

    # 若DEV_CONTAINER容器存在则删除容器并返回
    info "Remove existing Apollo Development container ..."
    remove_container_if_exists ${DEV_CONTAINER}

    # 检查主机GPU使用可用
    info "Determine whether host GPU is available ..."
    determine_gpu_use_host
    info "USE_GPU_HOST: ${USE_GPU_HOST}"

    # 检查设备是否存在并且建立数据卷的参数local_volumes
    local local_volumes=
    setup_devices_and_mount_local_volumes local_volumes

    # 拉取并运行所有的map容器，并将其中的地图文件保存到数据卷当中
    mount_map_volumes
    # 拉取并运行所有的需要的容器，并将其中的文件保存至数据卷当中
    mount_other_volumes

    info "Starting Docker container \"${DEV_CONTAINER}\" ..."

    local local_host="$(hostname)"
    local display="${DISPLAY:-:0}"
    local user="${USER}"
    local uid="$(id -u)"
    local group="$(id -g -n)"
    local gid="$(id -g)"

    # -x    执行指令后，会先显示该指令及所有的参数
    set -x

    # 启动DEV_CONTAINER容器
    # -itd会使容器一直保持后台运行，除非stop容器
    #--privileged使容器中的用户拥有真正的root权限
    # 设置容器中的环境变量
    # 将保存了数据的数据卷挂载到容器的相应位置
    # 将map容器的数据卷挂载到DEV_CONTAINER容器当中
    # 将其他容器的数据卷挂载到DEV_CONTAINER容器当中
    # 其他数据卷挂载
    # –net=host 共享主机网络空间，可以像普通进程一样进行网络通信
    # 设置工作空间
    # --add-host相当于在/ect/hosts添加解析的IP地址
    # 设置容器当中的主机名
    ${DOCKER_RUN_CMD} -itd \
        --privileged \
        --name "${DEV_CONTAINER}" \
        -e DISPLAY="${display}" \
        -e DOCKER_USER="${user}" \
        -e USER="${user}" \
        -e DOCKER_USER_ID="${uid}" \
        -e DOCKER_GRP="${group}" \
        -e DOCKER_GRP_ID="${gid}" \
        -e DOCKER_IMG="${DEV_IMAGE}" \
        -e USE_GPU_HOST="${USE_GPU_HOST}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
        ${MAP_VOLUMES_CONF} \
        ${OTHER_VOLUMES_CONF} \
        ${local_volumes} \
        --net host \
        -w /apollo \
        --add-host "${DEV_INSIDE}:127.0.0.1" \
        --add-host "${local_host}:127.0.0.1" \
        --hostname "${DEV_INSIDE}" \
        --shm-size "${SHM_SIZE}" \
        --pid=host \
        -v /dev/null:/dev/raw1394 \
        "${DEV_IMAGE}" \
        /bin/bash

    # $? 表示上个命令的退出的状态
    if [ $? -ne 0 ]; then
        error "Failed to start docker container \"${DEV_CONTAINER}\" based on image: ${DEV_IMAGE}"
        exit 1
    fi
    
    set +x

    postrun_start_user "${DEV_CONTAINER}"

    ok "Congratulations! You have successfully finished setting up Apollo Dev Environment."
    ok "To login into the newly created ${DEV_CONTAINER} container, please run the following command:"
    ok "  bash docker/scripts/dev_into.sh"
    ok "Enjoy!"
}

main "$@"
