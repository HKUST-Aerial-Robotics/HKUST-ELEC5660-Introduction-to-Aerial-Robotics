#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

CMD="/system/supervisor.sh"
if [ "$#" -gt 0 ]; then
    CMD="$1"
    shift
fi

docker run -it --rm \
    --net=host \
    -v $SCRIPT_DIR/../../../assignment:/ws \
    elec5660:2026 \
    "$CMD" "$@"
