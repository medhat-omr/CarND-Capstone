#!/usr/bin/env sh

# This script installs training environment on Ubuntu / Mac OS platforms
# It can be run locally or remotely on AWS server by aws_*.sh scripts
#
# To make sure all components are installed correctly, run it with sudo rights

conda create -n carnd-capstone python=3.6
source activate carnd-capstone

if hash apt-get 2>/dev/null; then
    # Ubuntu configuration
    pip install -r requirements_ubuntu.txt
    apt-get install protobuf-compiler
else
    # Mac OS configuration
    pip install -r requirements_macos.txt
    brew install protobuf-compiler
fi

# Workaround to enable training scripts outside Jupyter notebook
echo backend:TkAgg >~/.matplotlib/matplotlibrc

git clone https://github.com/dimaga/models

cd models/research
protoc object_detection/protos/*.proto --python_out=.
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim:`pwd`/object_detection