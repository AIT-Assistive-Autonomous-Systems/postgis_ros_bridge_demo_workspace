#!/bin/bash
set -e

./setup.sh
mkdir -p build install log
chown postgres:postgres build install log -R
su -c ./build.sh postgres
su -c ./test.sh postgres
