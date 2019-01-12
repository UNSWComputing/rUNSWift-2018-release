#!/usr/bin/python3

import os, re

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    return [ atoi(c) for c in re.split('(\d+)', text) ]


folder_dir="../Right Goal Filtered (Top Camera)"
path=folder_dir[3:]
for file in sorted(os.listdir(folder_dir), key=natural_keys):
    if file[-4:] == ".bmp":
        print(os.path.join("D:/ToR",path,file))
    
