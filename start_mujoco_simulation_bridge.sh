#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configure the ROS 2 domain for this simulation stack here.
export ROS_DOMAIN_ID=70
export ROS_LOG_DIR="${ROS_LOG_DIR:-${SCRIPT_DIR}/log/ros}"
export LD_LIBRARY_PATH="${SCRIPT_DIR}/build:${SCRIPT_DIR}/mj/install/custom_msgs/lib:${SCRIPT_DIR}/mj/src/custom_msgs/install/custom_msgs/lib:${SCRIPT_DIR}/mj/install/mujoco_node/lib:${SCRIPT_DIR}/mj/src/mujoco_node/install/mujoco_node/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"

if [[ -z "${MUJOCO_ROOT:-}" && -d "/home/huahui/zzy/mujoco-3.2.7" ]]; then
    export MUJOCO_ROOT="/home/huahui/zzy/mujoco-3.2.7"
fi

if [[ -n "${MUJOCO_ROOT:-}" ]]; then
    for mujoco_lib_dir in "${MUJOCO_ROOT}/build/lib" "${MUJOCO_ROOT}/lib" "${MUJOCO_ROOT}/bin"; do
        if [[ -d "${mujoco_lib_dir}" ]]; then
            export LD_LIBRARY_PATH="${mujoco_lib_dir}:${LD_LIBRARY_PATH}"
        fi
    done
fi

usage() {
    echo "用法: $0 <mujoco|bridge|control|kill|all> [程序参数...]"
    echo "  mujoco   启动 ROS2 MuJoCo 节点，发布 joint_states / imu_data"
    echo "  bridge   启动 mujoco_simulation_bridge，桥接 ROS2 和 DDS"
    echo "  control  启动 efc_app 运控程序"
    echo "  kill     清理 mujoco_node / mujoco_simulation_bridge / efc_app 旧进程"
    echo "  all      显示推荐的三个终端启动顺序"
}

kill_one() {
    local name="$1"
    local pattern="(^|/)${name}([[:space:]]|$)"

    if pgrep -x "${name}" >/dev/null 2>&1 || pgrep -f "${pattern}" >/dev/null 2>&1; then
        echo "[启动脚本] 清理旧进程: ${name}"
        pkill -TERM -x "${name}" || true
        pkill -TERM -f "${pattern}" || true
        sleep 0.5
    fi

    if pgrep -x "${name}" >/dev/null 2>&1 || pgrep -f "${pattern}" >/dev/null 2>&1; then
        echo "[启动脚本] 强制清理残留进程: ${name}"
        pkill -KILL -x "${name}" || true
        pkill -KILL -f "${pattern}" || true
        sleep 0.2
    fi
}

kill_all() {
    kill_one "mujoco_node"
    kill_one "mujoco_simulation_bridge"
    kill_one "efc_app"
}

find_executable() {
    local target="$1"

    if [[ -x "${SCRIPT_DIR}/build/${target}" ]]; then
        echo "${SCRIPT_DIR}/build/${target}"
        return 0
    fi

    if [[ -x "${SCRIPT_DIR}/bin/${target}" ]]; then
        echo "${SCRIPT_DIR}/bin/${target}"
        return 0
    fi

    return 1
}

source_ros_env() {
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "/opt/ros/humble/setup.bash"
    fi

    if [[ -f "${SCRIPT_DIR}/mj/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${SCRIPT_DIR}/mj/install/setup.bash"
    elif [[ -f "${SCRIPT_DIR}/mj/src/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${SCRIPT_DIR}/mj/src/install/setup.bash"
    elif [[ -f "${SCRIPT_DIR}/mj/src/custom_msgs/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${SCRIPT_DIR}/mj/src/custom_msgs/install/setup.bash"
    fi
}

find_mujoco_node_executable() {
    if [[ -n "${MUJOCO_NODE_PATH:-}" && -x "${MUJOCO_NODE_PATH}" ]]; then
        echo "${MUJOCO_NODE_PATH}"
        return 0
    fi

    local candidate
    for candidate in \
        "${SCRIPT_DIR}/mj/build/mujoco_node/mujoco_node" \
        "${SCRIPT_DIR}/mj/install/mujoco_node/lib/mujoco_node/mujoco_node" \
        "${SCRIPT_DIR}/mj/src/build/mujoco_node/mujoco_node" \
        "${SCRIPT_DIR}/mj/src/mujoco_node/build/mujoco_node/mujoco_node" \
        "${SCRIPT_DIR}/mj/src/mujoco_node/install/mujoco_node/lib/mujoco_node/mujoco_node"; do
        if [[ -x "${candidate}" ]]; then
            echo "${candidate}"
            return 0
        fi
    done

    return 1
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

TARGET=""
case "$1" in
    mujoco|mujoco_node)
        TARGET="mujoco_node"
        ;;
    bridge|mujoco_simulation_bridge)
        TARGET="mujoco_simulation_bridge"
        ;;
    control|efc_app)
        TARGET="efc_app"
        ;;
    kill|stop|clean)
        kill_all
        exit 0
        ;;
    all)
        echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
        echo "DDS: efc_app 发布控制命令 domain 50，订阅机器人状态 domain 60"
        echo "推荐启动顺序（三个终端分别运行；每次启动会先清理对应旧进程）:"
        echo "  1. $0 mujoco"
        echo "  2. $0 bridge"
        echo "  3. $0 control"
        exit 0
        ;;
    -h|--help|help)
        usage
        exit 0
        ;;
    *)
        echo "错误: 未知启动目标 '$1'"
        usage
        exit 1
        ;;
esac

shift

mkdir -p "${SCRIPT_DIR}/log" "${ROS_LOG_DIR}"

source_ros_env
kill_one "${TARGET}"

if [[ "${TARGET}" == "mujoco_node" ]]; then
    EXECUTABLE="$(find_mujoco_node_executable)" || {
        echo "错误: 找不到可执行文件 mujoco_node"
        echo "请先编译: cd ${SCRIPT_DIR}/mj && colcon build --packages-select mujoco_node"
        echo "也可以指定: export MUJOCO_NODE_PATH=/path/to/mujoco_node"
        exit 1
    }
    cd "${SCRIPT_DIR}/mj/src/mujoco_node"
elif [[ "${TARGET}" == "mujoco_simulation_bridge" ]]; then
    EXECUTABLE="$(find_executable "${TARGET}")" || {
        echo "错误: 找不到可执行文件 ${TARGET}"
        echo "请先编译: cd ${SCRIPT_DIR} && /opt/cmake-3.27/bin/cmake --build build --target ${TARGET} -j4"
        exit 1
    }
    cd "${SCRIPT_DIR}"
else
    EXECUTABLE="$(find_executable "${TARGET}")" || {
        echo "错误: 找不到可执行文件 ${TARGET}"
        echo "请先编译: cd ${SCRIPT_DIR} && /opt/cmake-3.27/bin/cmake --build build --target ${TARGET} -j4"
        exit 1
    }
    cd "${SCRIPT_DIR}"
fi

echo "[启动脚本] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[启动脚本] MUJOCO_ROOT=${MUJOCO_ROOT:-未设置}"
echo "[启动脚本] 启动 ${TARGET}: ${EXECUTABLE}"

exec "${EXECUTABLE}" "$@"
