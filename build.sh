#!/bin/bash
rm -rf build/

./waf configure --board CubeOrange
./waf copter
