# This script sets up the software required to run simswift
function myecho() {
	echo 
	echo -n " [+] "
 	echo -ne '\033[32;1m'
	echo -n $@
	echo -e '\033[0m'
}

function myerror() {
	echo 
	echo -n " [!] "
	echo -ne '\033[31;1m'
	echo -n $@
	echo -e '\033[0m'
}

myecho This script downloads and installs the required software to run simswift.

# Check if runswift has been setup
if [ -z ${RUNSWIFT_CHECKOUT_DIR+x} ]; then
	myerror "\$RUNSWIFT_CHECKOUT_DIR is not set! Please run 'build_setup.sh' first."
	exit
fi
cd "$RUNSWIFT_CHECKOUT_DIR/../"

# Check operating system
if [[ "$OSTYPE" != "linux-gnu" ]]; then
	myerror "simswift is only supported on Linux!"
	exit
fi

# Get machine type (32bit / 64bit)
export MACHINE_TYPE=`uname -m`

# Find number of processors
if [[ "$OSTYPE" == "linux-gnu" ]]; then
 	export NPROC=$(nproc)
elif [[ "$OSTYPE" == "darwin"* ]]; then
  	# Mac OSX
  	export NPROC=$(sysctl -n hw.ncpu)
fi

# NOTE: This might not be necessary - if the script fails try doing this step.
# Edit .../sources.list file to include universe and multiverse repositories
#myecho "We need to fetch pre-requisite software from the universe and multiverse repositories. Uncomment the lines with 'universe' or 'multiverse' in your /etc/apt/sources.list file. Opening file now..."
#sudo gedit /etc/apt/sources.list

# Update software list
myecho "Updating software listings..."
sudo apt-get update

# Download prerequisites
myecho "Downloading/installing simulation software pre-requisites..."
sudo apt-get install g++ subversion cmake libfreetype6-dev libode-dev libsdl-dev ruby ruby-dev libdevil-dev libboost-dev libboost-thread-dev libboost-regex-dev libboost-system-dev qt4-default libqt4-opengl-dev
sudo apt-get install gcc-multilib g++-multilib
sudo apt-get install default-jdk default-jre

# Download modified roboviz / simspark from runswift server 
export ROBOVIZ=roboviz.tar.gz
export SIMSPARK=simspark.tar.gz

myecho "Downloading/extracting simulation software..."
if [ ! -d roboviz ]; then
	myecho "Downloading modified RoboViz..."
	wget --continue --timestamping http://runswift2.cse.unsw.edu.au/simulation/${ROBOVIZ}
fi

if [ ! -d simspark ]; then
	myecho "Downloading modified simspark..."
	wget --continue --timestamping http://runswift2.cse.unsw.edu.au/simulation/${SIMSPARK}
fi

if [ ! -d roboviz ]; then
	myecho "Extracting roboviz..."
	tar -zxf ${ROBOVIZ} roboviz
fi

if [ ! -d simspark ]; then
	myecho "Extracting simspark..."
	tar -zxf ${SIMSPARK} simspark
fi

myecho "Building simulation software..."
myecho "Building and installing roboviz..."
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
	sudo ./roboviz/scripts/build-linux64.sh || exit
else
	sudo ./roboviz/scripts/build-linux32.sh || exit
fi

myecho "Building and installing simspark..."
sleep 2
cd simspark/spark
if [ -d build ] ; then
    rm -rf build
fi
mkdir build
cd build
cmake .. || exit
(sudo make -j${NPROC} && sudo make install) || exit
sudo ldconfig

myecho "Building and installing rcssserver3d..."
sleep 2
cd ../../rcssserver3d
if [ -d build ] ; then
    rm -rf build
fi
mkdir build
cd build
cmake .. || exit
(sudo make -j${NPROC} && sudo make install) || exit
sudo ldconfig

# Add roboviz path
myecho "Adding roboviz to system path..."
if ! grep -q "roboviz" ~/.bashrc ; then
	if [ ${MACHINE_TYPE} == 'x86_64' ]; then
      		# 64-bit
      		export ROBOVIZ_PATH="\$RUNSWIFT_CHECKOUT_DIR/../roboviz/bin/linux-amd64/"
    	else
	      	# 32-bit
	      	export ROBOVIZ_PATH="\$RUNSWIFT_CHECKOUT_DIR/../roboviz/bin/linux-i586/"
	fi
    	echo export PATH=\"$ROBOVIZ_PATH:\$PATH\" >> ~/.bashrc
fi

# Prepare system to use simulation build
cd $RUNSIWFT_CHECKOUT_DIR
myecho "Preparing system for use of simulation build..."

# Apply CTC patch
myecho "Downloading and installing CTC patch..."
wget --continue --timestamping http://runswift2.cse.unsw.edu.au/simulation/ctc_patch.tar.gz
tar -zxf ctc_patch.tar.gz ctc_patch
sudo cp -r ctc_patch/sysroot_legacy $RUNSWIFT_CHECKOUT_DIR/ctc/.
rm -r ctc_patch*
# Delete old files
rm $RUNSWIFT_CHECKOUT_DIR/ctc/sysroot_legacy/usr/lib/python2.7/random.pyc
rm $RUNSWIFT_CHECKOUT_DIR/ctc/sysroot_legacy/usr/lib/python2.7/random.pyo

# Fix python bug
myecho "Fixing python bug..."
sudo ln -s /usr/lib/python2.7/plat-*/_sysconfigdata_nd.py /usr/lib/python2.7/

# Create nao symlink
myecho "Creating /home/nao symlink..."
if [ ! -d /home/nao ]; then
    sudo ln -s $RUNSWIFT_CHECKOUT_DIR/image/home/nao/ /home/nao
fi

# Set LD_LIBRARY_PATH
myecho "Setting LD_LIBRARY_PATH to point to boostlibs..."
if ! grep -q "boost_libs" ~/.bashrc ; then
    echo export LD_LIBRARY_PATH="$RUNSWIFT_CHECKOUT_DIR/ctc/boost_libs" >> ~/.bashrc
fi

# Set up logging folders
myecho "Setting up \"/var/volatile/runswift\" logging folder..."
sudo mkdir /var/volatile
sudo mkdir /var/volatile/runswift
sudo chmod a+wrx /var/volatile/runswift 

# Finish
myecho "All done. Please close all shells (or run \". ~/.bashrc\"). Upon opening a new shell, you can start roboviz by typing 'roboviz.sh' or rcssserver by typing 'rcssserver3d'."
