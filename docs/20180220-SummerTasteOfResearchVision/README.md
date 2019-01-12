# Taste of Research - Machine Learning: Ball Detection

This documents my Taste of Research project which was completed across 12 weeks from Nov 2017 - Feb 2018

## Prerequisites

To download:

* [caffe/ssd](https://github.com/weiliu89/caffe/tree/ssd) - The framework used
* [MobileNetSDD](https://github.com/chuanqi305/MobileNet-SSD) - Used only for test/train scripts
* [SqueezenetSDD](https://github.com/chuanqi305/Squeezenet-SSD) - Used for prototxt files
* [kitti-SSD](https://github.com/jinfagang/kitti-ssd) - Used for preprocessing scripts
* [YOLO](https://pjreddie.com/darknet/yolo/) - Used for automated bounding box
* [LabelImg](https://github.com/tzutalin/labelImg) - Used for manual labelling of bouding boxes


NOTE:
To extract coordinates from YOLO, add: 
```
printf("%d, %d, %d, %d \n", left, right, top, bot);
```
in line 232 in image.c


## Directories

* Preprocessing - image preprocessing components
* Training - training components
* Deploy - network deployment and evaluation


## Author

* *Vintony Padmadiredja*


## Acknowledgments

* UNSW RoboCup Soccer "rUNSWift" Team
* UNSW RoboCup @ Home Team
* Professor Claude Sammut
* UNSW Faculty of Engineering
