#!/bin/bash
set -ex
exec supervisord -c /system/supervisord.conf
