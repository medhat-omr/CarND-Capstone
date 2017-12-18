#!/usr/bin/env sh
BOSCH_DATASET_NAME=bosch_dataset.zip
BOSCH_DATASET_ID=195tf4MTzzZpO1Sft5RLY5E4B6OgkmykN
GDRIVE_DOWNLOAD_URL=https://drive.google.com/uc?export=download

curl -sc /tmp/gcokie "${GDRIVE_DOWNLOAD_URL}&id=${ggID}" >/dev/null  
CONFIRM_ID="$(awk '/_warning_/ {print $NF}' /tmp/gcokie)"  
curl -LOJb /tmp/gcokie "${GDRIVE_DOWNLOAD_URL}&confirm=${CONFIRM_ID}&id=${BOSCH_DATASET_ID}" -o ${BOSCH_DATASET_NAME}
unzip ${BOSCH_DATASET_NAME}
