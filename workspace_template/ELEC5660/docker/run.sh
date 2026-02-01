#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

CMD="/system/supervisor.sh"
if [ "$#" -gt 0 ]; then
    CMD="$1"
    shift
fi

docker run -it --rm \
    --net=host \
    --ipc=host \
    --privileged \
    -v /dev:/dev \
    -v $SCRIPT_DIR/../../../:/ws \
    elec5660:2026 \
    "$CMD" "$@"
