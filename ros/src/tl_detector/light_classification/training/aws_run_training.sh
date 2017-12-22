#!/usr/bin/env sh

function show_help {
    echo Syntax: ./run_training_on_aws.sh "keyname.pem" udacity-aws-server-address-without-username
    echo
    echo Example:
    echo ./run_training_on_aws.sh "demo.pem" ec2-54-201-21-164.us-west-2.compute.amazonaws.com
}

function run_training {
    echo ssh -q -i "$1" ubuntu@$2 'bash -s' < ./aws_server_scripts/to_be_run_on_aws.sh
    ssh -q -i "$1" ubuntu@$2 'bash -s' < ./aws_server_scripts/to_be_run_on_aws.sh
}

if [ $# != 2 ]; then
    show_help
else
    run_training $1 $2
fi

