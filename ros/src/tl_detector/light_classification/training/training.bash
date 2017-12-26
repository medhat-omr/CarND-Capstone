#!/bin/bash

# Runs training procedure on the current machine and folder
#
# To change number of iterations, update model.ckpt-<number> in this file
# and num_steps parameter in training.config
#
# To change base model, update fine_tune_checkpoint path in training model
# and corresponding file name with id in download_datasets.bash

source activate carnd-capstone

python ./models/research/object_detection/train.py --pipeline_config_path=training.config --train_dir=training_intermediate

rm -rf training_results

python ./models/research/object_detection/export_inference_graph.py --pipeline_config_path=training.config --trained_checkpoint_prefix=training_intermediate/model.ckpt-10000 --output_directory=training_results

cp -f ./training_results/frozen_inference_graph.pb training_results.pb
