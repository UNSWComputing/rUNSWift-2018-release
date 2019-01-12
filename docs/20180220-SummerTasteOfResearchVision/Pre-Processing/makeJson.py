#!/usr/bin/python3

import os, re, json
from xml.etree import cElementTree as ET
def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    return [ atoi(c) for c in re.split('(\d+)', text) ]


payload={}
folder_dir="soccer_ball_bot_camera"
counter=0
for file in sorted(os.listdir(folder_dir), key=natural_keys):
    num=re.search('(\d+)', file)
    data = []
    if file[-4:] == ".xml":
        f = open(os.path.join(folder_dir,file), 'r')
        xml_str = f.read()
        root = ET.fromstring(xml_str)
        obj = root.find('object').find('bndbox')
        data.append(obj.find('xmin').text)
        data.append(obj.find('ymin').text)
        data.append(obj.find('xmax').text)
        data.append(obj.find('ymax').text)
        payload[os.path.join(folder_dir, file[:-4]+".jpg")] = data
        # os.rename(os.path.join(folder_dir, file[:-4]+'.bmp'), os.path.join('soccer_ball/', 'image'+str(counter)+'.bmp'))
        counter+=1
    elif not os.path.exists(os.path.join(folder_dir, file[:-4]+".xml")):
        f = open(folder_dir+" - results.txt", 'r')
        yolo_output = f.readlines()
        for i in range(0, len(yolo_output)):
            if re.search(file, yolo_output[i]):
                coord = str(yolo_output[i+2])
                coord = coord.strip()
                data = coord.split(', ')
                payload[os.path.join(folder_dir, file)] = data
                # os.rename(os.path.join(folder_dir, file), os.path.join('soccer_ball/', 'image'+str(counter)+'.bmp'))
                counter+=1

with open(folder_dir+"_train.json", 'w') as outfile:
    json.dump(payload, outfile, sort_keys=True, indent=4, ensure_ascii=False)