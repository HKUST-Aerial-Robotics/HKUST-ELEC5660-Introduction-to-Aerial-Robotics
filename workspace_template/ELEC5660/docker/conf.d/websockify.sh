#!/bin/bash
PORT=${PORT:-6080}
exec websockify --web /usr/share/novnc "$PORT" localhost:5900
