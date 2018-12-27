#!/bin/bash
set -e
git pull > /dev/null
git submodule update --init --recursive
./drwatson_zubax_gnss.py $@
