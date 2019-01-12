# Building OpenCV
This is essentially very similar to Fraser's guide on Building Camera Driver. Thanks to Fraser for the help with the VM.

[Aldebaran's guide](http://doc.aldebaran.com/2-1/dev/tools/vm-setup.html) on setting up and using the NaoOS VM.
1. Install Virtual Box VM
2. Download the latest NaoOS image for Virtual Box from Aldebaran/SBR and import into Virtual Box
3. Login to VM using Login: nao Pass: nao
4. Download OpenCV within /home/nao/ or wherever in the VM you want
```
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.1 
cd ..
```
5. OpenCV requires CMAKE2.8.12. Tranfser the above script to VM using pscp on windows
```
pscp -P 2222 cmake-2.8.12.2-Linux-i386.sh nao@localhost:/home/nao/.
```
6. Run the script to install
7. I used ccmake as it was easier to turn on and off features. 
```
cd opencv
mkdir build
cd build
~/cmake-2.8.12.2-Linux-i386/bin/ccmake ..
```
8. Apply the following changes
```
CMAKE_INSTALL_PREFIX = /usr
WITH_CUDA = OFF
WITH_IPP = OFF
WITH_MATLAB = OFF
WITH_OPENCL = OFF
WITH_OPNECLAMDBLAS = OFF
WITH_OPENCLAMDFFT = OFF
```

Then hit 'c' to configure and 'g' to generate

9. 
```
make
sudo make install
```
10. 
```
pscp -P 2222 -r nao@localhost:/usr/include/opencv .
pscp -P 2222 -r nao@localhost:/usr/include/opencv2 .
pscp -P 2222 nao@localhost:/usr/lib/libopencv_* .
pscp -P 2222 nao@localhost:/usr/lib/python2.7/site-packages/cv2.so .
pscp -P 2222 -r nao@localhost:/usr/share/OpenCV share/.
```
Last step above is so the 2 opencv folder won't merge

11. Move them across to the VM
12. Now ssh into a robot to root
```
ssh root@<robot>
```
And delete all the old opencv files within include, lib and share

13. Exit ssh and copy everything across. 
```
scp -r opencv root@<robot>:/usr/include/ opencv
scp -r opencv root@<robot>:/usr/include/ opencv2
scp -r opencv root@<robot>:/usr/lib/ libopencv_*
scp -r opencv root@<robot>:/usr/lib/python2.7/site-packages/ cv2.so
scp -r opencv root@<robot>:/usr/share/ OpenCV
```
13. Now ssh back into a robot
14. Change permissions of library files
```
cd /usr/lib
chmod 755 libopencv_*
```
15. Now you should be able to run opencv3 on python on ssh terminal and when you check the version
```
python
>>> import cv2
>>> cv2.__version__
'3.3.1'
```
There are plenty of other opencv files installed within the VM. You can also port them over but so far have not seen the need.

Also, there will be no image output when using stuff like imshow() and libgtk2.0 is required and is a huge library that I have no clue on how to install on the NAO