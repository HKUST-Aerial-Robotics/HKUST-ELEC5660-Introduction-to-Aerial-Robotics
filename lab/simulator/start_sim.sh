#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
BASE_DIR=$SCRIPT_DIR

# Generate a unique project name based on the current timestamp
TIMESTAMP=$(date +%Y%m%d%H%M%S)
PROJECT_NAME="isaaclab_${TIMESTAMP}"
DOCKER_COMPOSE_FILE="$SCRIPT_DIR/docker/docker-compose.yml"

export BASE_DIR=$SCRIPT_DIR
export DISPLAY=$DISPLAY
export DOCKER_ISAACLAB_PATH=/workspace/isaaclab

# Function to clean up containers when script exits
cleanup() {
    echo "Cleaning up container: $PROJECT_NAME"
    docker compose -f $DOCKER_COMPOSE_FILE -p $PROJECT_NAME down
    echo "Container cleaned up successfully."
}

# Register the cleanup function to run on script exit
trap cleanup EXIT INT TERM

# Display help message
show_help() {
    echo "Usage: ./start_sim.sh [options] [command]"
    echo ""
    echo "This script starts a Docker container with Isaac Sim environment."
    echo ""
    echo "Options:"
    echo "  -h, --help     Display this help message and exit"
    echo "  --stop-all     Stop all running containers created by this script"
    echo "  --gui          Run with GUI window (no --headless)"
    echo ""
echo "If command arguments are provided, they will be passed to the simulator main.py."
echo "Example: ./start_sim.sh --headless --enable_cameras"
echo ""
echo "If no arguments are provided, the simulator runs headless with cameras enabled."
    echo ""
    echo "Each run creates a new container instance with a unique name."
    # Don't call exit directly as it would trigger cleanup
    trap - EXIT INT TERM
    exit 0
}

# Function to stop all containers
stop_all_containers() {
    echo "Stopping all isaaclab containers..."
    # Find all project names starting with isaaclab_
    PROJECTS=$(docker compose ls --format "{{.Project}}" | grep "^isaaclab_")

    if [ -z "$PROJECTS" ]; then
        echo "No running isaaclab containers found."
    else
        for PROJECT in $PROJECTS; do
            echo "Stopping project: $PROJECT"
            docker compose -p $PROJECT down
        done
        echo "All isaaclab containers stopped."
    fi
    # Don't call exit directly as it would trigger cleanup for the current container too
    trap - EXIT INT TERM
    exit 0
}

# Check for help option or stop-all command
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
elif [ "$1" = "--stop-all" ]; then
    stop_all_containers
fi

# Build entrypoint to run the simulator main
GUI_MODE=false
RUN_ARGS_LIST=()
for arg in "$@"; do
    if [ "$arg" = "--gui" ]; then
        GUI_MODE=true
    else
        RUN_ARGS_LIST+=("$arg")
    fi
done

if [ ${#RUN_ARGS_LIST[@]} -ge 1 ]; then
    RUN_ARGS="${RUN_ARGS_LIST[*]}"
else
    RUN_ARGS="--headless --enable_cameras"
fi

if [ "$GUI_MODE" = true ]; then
    # Strip any explicit --headless if present
    RUN_ARGS=$(echo "$RUN_ARGS" | sed "s/--headless//g")
    RUN_ARGS="--enable_cameras $RUN_ARGS"
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0
    fi
    xhost + >/dev/null 2>&1
fi

ENTRYPOINT_CMD="/bin/bash -c \"source ~/.bashrc && ISAACSIM_ROOT=/isaac-sim && export ROS_DISTRO=\\\${ROS_DISTRO:-jazzy} && export RMW_IMPLEMENTATION=\\\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp} && export LD_LIBRARY_PATH=\\\$LD_LIBRARY_PATH:\\\$ISAACSIM_ROOT/exts/isaacsim.ros2.bridge/\\\$ROS_DISTRO/lib && /workspace/isaaclab/_isaac_sim/python.sh /workspace/isaaclab/simulator/sim_server/main.py ${RUN_ARGS} --kit_args \\\"--enable isaacsim.ros2.bridge\\\"\""
export ENTRYPOINT="$ENTRYPOINT_CMD"
echo "Setting ENTRYPOINT to: $ENTRYPOINT"

# Create shared volumes if they don't exist
VOLUMES=(
    "isaac_shared_cache_kit"
    "isaac_shared_cache_ov"
    "isaac_shared_cache_pip"
    "isaac_shared_cache_gl"
    "isaac_shared_cache_compute"
    "isaac_shared_logs"
    "isaac_shared_carb_logs"
    "isaac_shared_data"
    "isaac_shared_docs"
    "isaac_shared_lab_docs"
    "isaac_shared_lab_logs"
    "isaac_shared_lab_data"
)

echo "Checking for shared volumes..."
for vol in "${VOLUMES[@]}"; do
    if ! docker volume ls -q -f name=^${vol}$ | grep -q .; then
        echo "Creating volume: $vol"
        docker volume create $vol
    else
        echo "Volume exists: $vol"
    fi
done

# xhost is only needed for GUI mode
mkdir -p $BASE_DIR/logs
echo "Starting new container with project name: $PROJECT_NAME"
# Use --detach to run in background, script will wait and clean up when done
docker compose -f $DOCKER_COMPOSE_FILE -p $PROJECT_NAME up
# The cleanup function will be called automatically when the script exits
