#!/bin/sh

for file in "Test Precision and Recall/positive/"*
do
	read -t 0.25 -N 1 input
	if test $input == "q" 
    then
		break
    fi
    python3 "deep_learning_object_detection.py" --prototxt model/miniSqueezeDet_deploy.prototxt \
    	--model model/miniSqueezeDet_iter_35263.caffemodel --image "$file"
done 
