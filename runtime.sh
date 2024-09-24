#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

CYCLONE_VOL=""
BASH_CMD=""

# Default cyclone_dds.xml path
CYCLONE_DIR=/home/$USER/cyclone_dds.xml

# Function to print usage
usage() {
    echo "
Usage: dev.sh [-b|bash] [-l|--local] [-h|--help]

Where:
    -b | bash       Open bash in docker container (Default in dev.sh)
    -l | --local    Use default local cyclone_dds.xml config
    -l | --local    Optionally point to absolute -l /path/to/cyclone_dds.xml
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

# Build docker image only up to base stage
DOCKER_BUILDKIT=1 docker build \
    -t lidar_image_pointcloud:latest \
    -f Dockerfile --target runtime .

# Run docker image without volumes
docker run -it --rm --net host --privileged \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v /etc/localtime:/etc/localtime:ro \
    $CYCLONE_VOL \
    lidar_image_pointcloud:latest $BASH_CMD
