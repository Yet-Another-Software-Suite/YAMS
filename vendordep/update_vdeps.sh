#!/bin/bash
cd ..
ls examples/*/vendordeps | grep vendordeps | awk -F':' '{print $1"/"}' | xargs -n1 cp vendordep/vendordeps/*
ls examples/* | grep ':' | awk -F':' '{print $1"/"}' | xargs -n1 cp examples/simple_robot/build.gradle 
