#!/usr/bin/env sh

# This script installs training environment on Ubuntu / Mac OS platforms
# It can be run locally or remotely on AWS server by aws_*.sh scripts
#
# To make sure all components are installed correctly, run it with sudo rights

conda create -n carnd-capstone python=3.6
source activate carnd-capstone
pip install -r requirements.txt

git clone https://github.com/tensorflow/models

if hash apt-get 2>/dev/null; then
    apt-get install protobuf-compiler
else
    brew install protobuf-compiler
fi

cd models/research
protoc object_detection/protos/*.proto --python_out=.
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim