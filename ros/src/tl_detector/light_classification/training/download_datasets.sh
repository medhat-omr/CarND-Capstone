#!/usr/bin/env sh

function download_file {
    FILE_NAME=$1
    FILE_ID=$2
    GDRIVE_DOWNLOAD_URL=https://drive.google.com/uc?export=download

    [ -e ${FILE_NAME} ] && rm ${FILE_NAME}
    curl -sc /tmp/gcokie "${GDRIVE_DOWNLOAD_URL}&id=${FILE_ID}" >/dev/null  
    CONFIRM_ID="$(awk '/_warning_/ {print $NF}' /tmp/gcokie)"  
    curl -LOJb /tmp/gcokie "${GDRIVE_DOWNLOAD_URL}&confirm=${CONFIRM_ID}&id=${FILE_ID}"
}

download_file dataset_sdcnd_capstone_sim_data.record 18eWSAR1Bgm7N0QKJcKyF9Y8dt_J-Olrf
download_file dataset_sdcnd_capstone_real_data.record 16O9VUYBrioOASNXnR7nt-_dcbh3KPlvy
download_file bosch_dataset.record 1ird1oLMEWndnbm3cDYHKju1li1P0QUhO
download_file ssd_mobilenet_v1_coco_2017_11_17.tar 1DFjo_cgAYW6Cchvryu6obWKIx6KWwJnK
tar -xvzf ssd_mobilenet_v1_coco_2017_11_17.tar