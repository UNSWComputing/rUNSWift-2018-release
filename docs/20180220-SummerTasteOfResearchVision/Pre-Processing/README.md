# Data Preprocessing


## Collect Raw Images

Use the NAO robots to record dumps with the soccer ball. Try to obtain images in various different conditions such as angles, lighting, object's surrounding it (robots, goal posts, field lines), etc. Extract the raw images from offnao.

## Adding Bounding Boxes

### YOLO
Use YOLO to automate the bounding box process. Create an image_lists.txt file using makeImageList.py. Then run YOLO using this:

```
./darknet detector test data/coco.data yolo.cfg yolo.weights < image_list.txt > result.txt
```
These detections might result to more than one bounding box. Use findDoubleLabel.py to find images containing more than 1 label. Then manual remove these lines from results.txt

### LabelImg
YOLO might've not been able to automaticall create bounding boxes for all images, such as blurred balls. Run getUnlabelledImages.sh which will move images without a label after modifying to your desired directories in the script.

Open a folder in LabelImg and start labelling the images. This will take some time. LabelImg produces the labels in an xml file.

## Formating Labels

### Merging Labels
I decided to combined both YOLO and LabelImg labels to a single JSON file. Run makeJson.py.

### Convert Labels
Next, we convert the labels a single txt for each. Run convert.py

### Create LMDB Database
Lastly, create an LMDB Database following the steps [here](https://github.com/jinfagang/kitti-ssd).
