# Deployment

Copy over the \*_deploy.prototxt and \*\_iter\_\*.caffemodel to one of these directories

NOTE:
Requires OPENCV3.3+, check OPENCV folder from root for installation to NAO

By copying these folders to one of the NAOs after installing OPENCV3, you can test the runtime of your network. 

## Python

Run by
```
python object_detect.py --prototxt model/miniSqueezenet_depoly.prototxt --model model/miniSqueezenet_iter_100000.caffemodel --image images/Image1.jpg
```
Python script modified from [pyimagesearch](https://www.pyimagesearch.com/2017/09/11/object-detection-with-deep-learning-and-opencv/)

## C++

Compile and run by
```
./ObjectDetect -proto=model/miniSqueezenet_depoly.prototxt -model=model/miniSqueezenet_iter_100000.caffemodel -video=images/Image1.jpg
```
C++ script modified from [opencv](https://github.com/opencv/opencv/blob/master/samples/dnn/ssd_object_detection.cpp)

### BallDetector.c

BallDetector.c is my attempt to compile the C++ code within rUNSWift, however having difficuly compiling with OpenCV

## Analysis
| Network         | Speed C++ NAO | Speed Python NAO  | mAP           |   
| --------------- | ------------- | ----------------- | ------------- |   
| Squeeznet       | ~88ms         | ~95ms             | 86.7%         |   
| MiniSqueezenet  | ~20ms         | ~25ms             | 58.0%         | 
| MiniSqueezeDet  | ~35ms         | ~40ms             | 65.0%         |   

## Precision & Recall
Precision and recall was calculated using a sample of 100 positive and negative new data. Can be found within dataset.zip

### MiniSqueezenet

#### 30% confidence threshold
| TP | FN | FP | TN |
| -- | -- | -- | -- |
| 92 | 7  | 37 | 64 |


Precision = TP/(TP+FP) = 0.71318

Recall = TP/(TP+FN) = 0.92930


#### 40% confidence threshold
| TP | FN | FP | TN |
| -- | -- | -- | -- |
| 87 | 13 | 11 | 89 |


Precision = TP/(TP+FP) = 0.88776

Recall = TP/(TP+FN) = 0.87


#### 50% confidence threshold
| TP | FN | FP | TN |
| -- | -- | -- | -- |
| 74 | 26 | 6  | 94 |


Precision = TP/(TP+FP) = 0.925

Recall = TP/(TP+FN) = 0.74


### MiniSqueezeDet
#### 40% confidence threshold
| TP | FN | FP | TN |
| -- | -- | -- | -- |
| 83 | 17 | 4  | 96 |


Precision = TP/(TP+FP) = 0.95402

Recall = TP/(TP+FN) = 0.83


#### 30% confidence threshold
| TP | FN | FP | TN |
| -- | -- | -- | -- |
| 86 | 13 | 23 | 78 |


Precision = TP/(TP+FP) = 0.78899

Recall = TP/(TP+FN) = 0.86869