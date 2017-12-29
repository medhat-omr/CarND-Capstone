#!/bin/bash

# Installs training environment, downloads datasets and runs training procedure
# on AWS server. The result of the training, training_resupts.pb is downloaded
# from AWS server after the training is complete.
#
# The script is launched from the local machine. pem-key and AWS server address
# with udacity-carnd-advanced-deep-learning configuration should be passed
# as its command line arguments.

function show_help {
    echo Usage example: ./aws_install_download_and_run.bash "demo.pem" ec2-54-201-21-164.us-west-2.compute.amazonaws.com
}

if [ $# != 2 ]; then
    show_help
else
    ./aws_install_and_download.bash $1 $2
    ./aws_run_training.bash $1 $2
fi

