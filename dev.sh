#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

CYCLONE_VOL=""
BASH_CMD=""

# Default cyclone_dds.xml path
CYCLONE_DIR=/home/$USER/cyclone_dds.xml
EXPORTS_OUTPUT_DIR=$HOME/exported_data

# Default value for headless
headless=false

# Function to print usage
usage() {
    echo "
Usage: dev.sh [-b|bash] [-l|--local] [-h|--help]

Where:
    -b | bash       Open bash in docker container (Default in dev.sh)
    -l | --local    Use default local cyclone_dds.xml config
    -l | --local    Optionally point to absolute -l /path/to/cyclone_dds.xml
    -o | --output   EXPORTS_OUTPUT_DIR
    -h | --help     Show this help message
    "
    exit 1
}

# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -b|bash)
            BASH_CMD=bash
            ;;
        -l|--local)
            # Avoid getting confused if bash is written where path should be
            if [ -n "$2" ]; then
                if [ ! $2 = "bash" ]; then
                    CYCLONE_DIR="$2"
                    shift
                fi
            fi
            CYCLONE_VOL="-v $CYCLONE_DIR:/opt/ros_ws/cyclone_dds.xml"
            ;;
        -o|--output)
            if [[ -n "$2" && "$2" != -* ]]; then
                EXPORTS_OUTPUT_DIR="$2"
                shift
            else
                echo "Error: Argument for $1 is missing."
                usage
            fi
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
    shift
done

# Verify CYCLONE_DIR exists
if [ -n "$CYCLONE_VOL" ]; then
    if [ ! -f "$CYCLONE_DIR" ]; then
        echo "$CYCLONE_DIR does not exist! Please provide a valid path to cyclone_dds.xml"
        exit 1
    fi
fi

# Verify EXPORTS_OUTPUT_DIR exists
if [ ! -d "$EXPORTS_OUTPUT_DIR" ]; then
    echo "$EXPORTS_OUTPUT_DIR does not exist! Please provide a valid path where exported data is stored"
    exit 1
fi

# Build docker image up to dev stage
docker build \
    -t lidar_image_pointcloud:latest-dev \
    -f Dockerfile --target dev .


MOUNT_X=""
if [ "$headless" = "false" ]; then
    MOUNT_X="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix"
    xhost +local:root
    # xhost + >/dev/null
fi

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    ${MOUNT_X} \
    -e XAUTHORITY="${XAUTHORITY}" \
    -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v /etc/localtime:/etc/localtime:ro \
    -v ./lidar_image_pointcloud:/opt/ros_ws/src/lidar_image_pointcloud \
    $CYCLONE_VOL \
    -v $EXPORTS_OUTPUT_DIR:/opt/ros_ws/exported_data \
    lidar_image_pointcloud:latest-dev $BASH_CMD
