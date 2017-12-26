#!/bin/bash

# Installs training environment and downloads datasets on AWS server
#
# The script is launched from the local machine. pem-key and AWS server address
# with udacity-carnd-advanced-deep-learning configuration should be passed
# as its command line arguments.

function show_help {
    echo Usage example: ./aws_install_and_download.bash "demo.pem" ec2-54-201-21-164.us-west-2.compute.amazonaws.com
}

function install_and_download {
    AWS_PARAMS="-i "$1" ubuntu@$2"
    TRAINING_FOLDER=training
   
    ssh ${AWS_PARAMS} "mkdir ${TRAINING_FOLDER}"
    ssh ${AWS_PARAMS} "cd ~/${TRAINING_FOLDER} && bash -s" <install_environment.bash
    ssh ${AWS_PARAMS} "cd ~/${TRAINING_FOLDER} && bash -s" <download_datasets.bash
}

if [ $# != 2 ]; then
    show_help
else
    install_and_download $1 $2
fi

