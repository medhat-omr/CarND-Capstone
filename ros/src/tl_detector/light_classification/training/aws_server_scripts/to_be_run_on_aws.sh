#!/usr/bin/env sh

function unpack_zip_from_gdrive {
    FILE_NAME=$1
    FILE_ID=$2
    GDRIVE_DOWNLOAD_URL=https://drive.google.com/uc?export=download

    [ -e ${FILE_NAME} ] && rm ${FILE_NAME}
    curl -sc /tmp/gcokie "${GDRIVE_DOWNLOAD_URL}&id=${FILE_ID}" >/dev/null  
    CONFIRM_ID="$(awk '/_warning_/ {print $NF}' /tmp/gcokie)"  
    curl -LOJb /tmp/gcokie "${GDRIVE_DOWNLOAD_URL}&confirm=${CONFIRM_ID}&id=${FILE_ID}"
    unzip ${FILE_NAME}
}

cd ~/CarND-Capstone/ros/src/tl_detector/light_classification/training

unpack_zip_from_gdrive bosch-dataset.zip 195tf4MTzzZpO1Sft5RLY5E4B6OgkmykN
unpack_zip_from_gdrive dataset-sdcnd-capstone.zip 168a_xIMaVoMSaC4ayvzbZjizmAp77mJi

python run_training.py