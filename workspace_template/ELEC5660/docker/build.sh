#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

docker build -t elec5660:2026 $SCRIPT_DIR