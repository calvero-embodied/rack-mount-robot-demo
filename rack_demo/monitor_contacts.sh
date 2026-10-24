#!/bin/bash
set -e
cd /home/ubuntu/rack_demo
if [ ! -x contact_monitor ] || [ contact_monitor.cc -nt contact_monitor ]; then
    g++ contact_monitor.cc -o contact_monitor $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8) -pthread
fi
exec ./contact_monitor contact_topics.txt contact_audit.json
