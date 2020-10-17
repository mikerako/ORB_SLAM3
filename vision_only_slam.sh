#!/bin/bash
pathDatasetVSlam='/home/aakash/Documents/ORB_SLAM3/Datasets/vSLAM' #Example, it is necesary to change it by the dataset path


echo "Launching Vision Only Slam"
./Examples/467/mono_mbot ./Vocabulary/ORBvoc.txt ./Examples/467/mbot_params.yaml "$pathDatasetVSlam"/images "$pathDatasetVSlam"/timestamps.txt dataset-vSLAM

