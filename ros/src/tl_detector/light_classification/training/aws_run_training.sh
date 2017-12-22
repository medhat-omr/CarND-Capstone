#!/usr/bin/env sh

function show_help {
    echo Usage example: ./run_training_on_aws.sh "demo.pem" ec2-54-201-21-164.us-west-2.compute.amazonaws.com
}

function run_training {
    GITHUB_URL="$(git config --get remote.origin.url | sed 's/git@github.com:/https:\/\/github.com\//')"
    GIT_BRANCH="$(git branch | grep \* | cut -d ' ' -f2)"
    AWS_PARAMS="-i "$1" ubuntu@$2"
    TRAINING_FOLDER="~/CarND-Capstone/ros/src/tl_detector/light_classification/training"

    ssh ${AWS_PARAMS} "git clone ${GITHUB_URL}"
    ssh ${AWS_PARAMS} "cd CarND-Capstone && git checkout -b ${GIT_BRANCH} origin/${GIT_BRANCH}"
    ssh ${AWS_PARAMS} 'bash -s' < ./aws_server_scripts/to_be_run_on_aws.sh
    ssh ${AWS_PARAMS} "cd ${TRAINING_FOLDER} && ./run_training.py"

    sftp ${AWS_PARAMS}:${TRAINING_FOLDER}/training_results.pb .
}

if [ $# != 2 ]; then
    show_help
else
    run_training $1 $2
fi

