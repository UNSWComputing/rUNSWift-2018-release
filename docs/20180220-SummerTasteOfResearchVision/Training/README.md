# Training

Set up a directory where you want to train your data.
Copy:
* solver_test.prototxt	
* solver_train.prototxt	
* test.sh	
* train.sh

from [MobileNet-SSD](https://github.com/chuanqi305/MobileNet-SSD) to your directory

## Create symbolic link to lmdb

```
ln -s PATH_TO_YOUR_TRAIN_LMDB trainval_lmdb
ln -s PATH_TO_YOUR_TEST_LMDB test_lmdb
```
Within your directory

## Modifying scritps

Remove line 9 and modify line 2 accordingly from train.sh 

Modify line 1, 2, 12 accordingly from solver_train.protxt

If running multiple gpus, change line 10 in train.sh to -gpu 0,1 (for 2 GPU)

## Train

Train network using one of my configurations:
* Modified Squeezenet
* Modified MiniSqueeznet
* MiniSqueezeDet
or make your own

Train by running train.sh

## Test
Use test.sh to evaluate your network. Modify lines accordingly - similar to train files.