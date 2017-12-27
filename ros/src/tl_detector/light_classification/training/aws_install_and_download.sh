#!/usr/bin/env sh

function show_help {
    echo Usage example: ./aws_install_and_download.sh "demo.pem" ec2-54-201-21-164.us-west-2.compute.amazonaws.com
}

function install_and_download {
    AWS_PARAMS="-i "$1" ubuntu@$2"
    TRAINING_FOLDER=training
   
    ssh ${AWS_PARAMS} "mkdir ${TRAINING_FOLDER}"
    ssh ${AWS_PARAMS} "cd ~/${TRAINING_FOLDER} && bash -s" <install_environment.sh
    ssh ${AWS_PARAMS} "cd ~/${TRAINING_FOLDER} && bash -s" <download_datasets.sh
}

if [ $# != 2 ]; then
    show_help
else
    install_and_download $1 $2
fi

