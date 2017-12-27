#!/usr/bin/env sh

function show_help {
    echo Usage example: ./aws_run_training.sh "demo.pem" ec2-54-201-21-164.us-west-2.compute.amazonaws.com
}

function run_training {
    TRAINING_FOLDER=training

    scp training.config ${AWS_PARAMS}:${TRAINING_FOLDER}
    ssh ${AWS_PARAMS} "cd ~/${TRAINING_FOLDER} && bash -s"<training.sh
    scp ${AWS_PARAMS}:${TRAINING_FOLDER}/training_results.pb .
}

if [ $# != 2 ]; then
    show_help
else
    run_training $1 $2
fi

