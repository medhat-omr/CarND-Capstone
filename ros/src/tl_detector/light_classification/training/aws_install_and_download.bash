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
    PEM="-i "$1""
    SERVER="ubuntu@$2"
    TRAINING_FOLDER=training
   
    ssh ${PEM} ${SERVER} "mkdir ${TRAINING_FOLDER}"

    scp ${PEM} install_environment.bash ${SERVER}:${TRAINING_FOLDER}
    scp ${PEM} requirements_ubuntu.txt ${SERVER}:${TRAINING_FOLDER}
    ssh ${PEM} ${SERVER}
    #ssh ${PEM} ${SERVER} "cd ~/${TRAINING_FOLDER} && ./install_environment.bash"

    scp ${PEM} download_datasets.bash ${SERVER}:${TRAINING_FOLDER}
    ssh ${PEM} ${SERVER} "cd ~/${TRAINING_FOLDER} && ./download_datasets.bash"
}

if [ $# != 2 ]; then
    show_help
else
    install_and_download $1 $2
fi

