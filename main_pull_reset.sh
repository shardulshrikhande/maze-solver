#!/bin/bash
git pull
git reset --hard origin/main
find ./ -name '*py' -exec chmod +x '{}' \;
chmod +x main_pull_reset.sh
chmod +x make_executable.sh