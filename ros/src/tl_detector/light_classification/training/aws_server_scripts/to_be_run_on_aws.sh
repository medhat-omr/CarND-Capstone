#!/usr/bin/env sh
BOSCH_DATASET_NAME=bosch-dataset.zip
BOSCH_DATASET_ID=195tf4MTzzZpO1Sft5RLY5E4B6OgkmykN
GDRIVE_DOWNLOAD_URL=https://drive.google.com/uc?export=download

[ -e ${BOSCH_DATASET_NAME} ] && rm ${BOSCH_DATASET_NAME}
curl -sc /tmp/gcokie "${GDRIVE_DOWNLOAD_URL}&id=${BOSCH_DATASET_ID}" >/dev/null  
CONFIRM_ID="$(awk '/_warning_/ {print $NF}' /tmp/gcokie)"  
curl -LOJb /tmp/gcokie "${GDRIVE_DOWNLOAD_URL}&confirm=${CONFIRM_ID}&id=${BOSCH_DATASET_ID}"
unzip ${BOSCH_DATASET_NAME}
