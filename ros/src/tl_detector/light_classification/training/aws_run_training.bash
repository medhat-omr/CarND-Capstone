#!/bin/bash

# Runs training procedure on AWS server. The result of the training,
# training_resupts.pb is downloaded from AWS server after the training is
# complete.
#
# The script is launched from the local machine. pem-key and AWS server address
# with udacity-carnd-advanced-deep-learning configuration should be passed
# as its command line arguments.

function show_help {
    echo Usage example: ./aws_run_training.bash "demo.pem" ec2-54-201-21-164.us-west-2.compute.amazonaws.com
}

function run_training {
    PEM="-i "$1""
    SERVER="ubuntu@$2"
    TRAINING_FOLDER=training

    scp ${PEM} training.config ${SERVER}:${TRAINING_FOLDER}
    scp ${PEM} training.bash ${SERVER}:${TRAINING_FOLDER}
    ssh ${PEM} ${SERVER}
    #ssh ${PEM} ${SERVER} "cd ~/${TRAINING_FOLDER} && ./training.bash"
    scp ${PEM} ${SERVER}:${TRAINING_FOLDER}/training_results.pb .
}

if [ $# != 2 ]; then
    show_help
else
    run_training $1 $2
fi

