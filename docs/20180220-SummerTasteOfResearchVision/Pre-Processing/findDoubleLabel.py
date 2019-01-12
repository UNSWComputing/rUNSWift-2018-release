#!/usr/bin/python3

import re

f = open("soccer_ball_top_new - results.txt", 'r')
yolo_output = f.readlines()
for i in range(0, len(yolo_output)):
    if re.search('sports ball:', yolo_output[i]) and re.search('sports ball:', yolo_output[i+2]):
        print(yolo_output[i-1], end='')