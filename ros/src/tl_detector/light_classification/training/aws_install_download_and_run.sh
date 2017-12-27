#!/usr/bin/env sh

function show_help {
    echo Usage example: ./aws_install_download_and_run.sh "demo.pem" ec2-54-201-21-164.us-west-2.compute.amazonaws.com
}

if [ $# != 2 ]; then
    show_help
else
    ./aws_install_and_download.sh
    ./aws_run_training.sh
fi

