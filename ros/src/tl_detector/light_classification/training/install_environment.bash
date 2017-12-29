#!/bin/bash

# Installs training environment to the machine and folder the script is run
# from

conda create -n carnd-capstone python=3.6
source activate carnd-capstone

if hash apt-get 2>/dev/null; then
    # Ubuntu configuration
    pip install -r requirements_ubuntu.txt
    
    # Magic tricks to work around problems of installation problems
    # of protobuf-compiler on udacity-carnd-advanced-deep-learning
    # configuration on AWS
    sudo rm /var/lib/dpkg/lock
    sudo dpkg --configure -a
    sudo dpkg --configure -a # First attempt usually fails :)

    sudo apt-get install protobuf-compiler
else
    # Mac OS configuration
    pip install -r requirements_macos.txt
    brew install protobuf-compiler

    # Workaround to enable training scripts outside Jupyter notebook
    echo backend:TkAgg >~/.matplotlib/matplotlibrc
fi

# Unlike tensorflow/models, which may also change in the future,
# dimaga/models has compatibility fixes with TL 1.3, while supporting
# multiple *.record files - relatively new feature
git clone https://github.com/dimaga/models
cd models/research
protoc object_detection/protos/*.proto --python_out=.
