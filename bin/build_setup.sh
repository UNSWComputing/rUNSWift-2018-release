#!/bin/bash

# OSX setup
#  1. Move this to runswift wiki? [After coderelease because possible wiki merge conflicts]
#  2. Put ${OSX_CTC_ZIP} into rUNSWift/ctc directory.
#     # You have to get it from Aldebaran at this time.
#  3. bin/build_setup.sh  # Should build libagent and runswift
#  4. nao_sync to the robot as under Linux, e.g. nao_sync -ar nao.local
#  5. offnao does not work with OSX yet, it is not clear
#     whether it will be easier to upgrade to Qt5 and clean up offnao, or
#     restart from scratch in say Python, or
#     build a Team Communication Monitor plugin
#  6. simswift does not work on macOS
#  7. vatnao and vatnao-legacy do not work on macOS

set -e
set -u

function myecho() {
  echo -n " [+] "
  echo -ne '\033[32;1m'
  echo -n $@
  echo -e '\033[0m'
}

OLD_RCD="${RUNSWIFT_CHECKOUT_DIR-}"
unset RUNSWIFT_CHECKOUT_DIR
unset CTC_DIR

# Set up git
cat << USER_CONFIG
If the user info is incorrect, please configure it like:
  git config user.name Name
  git config user.email email@address
USER_CONFIG
echo Your user name: $(git config user.name)
echo Your email: $(git config user.email)

# Set up ssh_config
mkdir -p ~/.ssh
for host in robot1 robot2 robot3 robot4 robot5 robot6
do
  if [ ! -f ~/.ssh/config ] || ! grep -q "Host $host" ~/.ssh/config ; then (
      echo "Host $host"
      echo "  Hostname $host.local"
      echo "  HostKeyAlias $host"
      echo "  CheckHostIP no"
      echo "  User nao"
      echo
    ) >> ~/.ssh/config
  fi
done

## TODO: remove lines from bashrc from old runswift stuff, path, etc.

# Set up bash
# Allow to be run as either `cd bin;./build_setup.sh` OR `./bin/build_setup.sh`
export RUNSWIFT_CHECKOUT_DIR=${PWD///bin/}
echo RUNSWIFT_CHECKOUT_DIR is $RUNSWIFT_CHECKOUT_DIR
if ! grep -q "# Robocup stuff" ~/.bashrc ; then (
echo >> ~/.bashrc
echo "# Robocup stuff" >> ~/.bashrc
echo export RUNSWIFT_CHECKOUT_DIR=\"$RUNSWIFT_CHECKOUT_DIR\" >> ~/.bashrc
echo export PATH=\"\$RUNSWIFT_CHECKOUT_DIR/bin:\$PATH\" >> ~/.bashrc
)
fi
if [[ x"$OLD_RCD" != x"$RUNSWIFT_CHECKOUT_DIR" ]]; then
  trap "myecho RUNSWIFT_CHECKOUT_DIR has changed from \'$OLD_RCD\' to \'$RUNSWIFT_CHECKOUT_DIR\'.  please be sure to reload ~/.bashrc before fixing things manually" ERR
fi

bin/gamecontroller_install.sh

# SSH keys
ssh-keygen -l -f ~/.ssh/id_rsa.pub > /dev/null || ssh-keygen
if ! grep -qf ~/.ssh/id_rsa.pub "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys; then
  echo >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
  echo "# $(git config user.name)'s key" >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
  cat ~/.ssh/id_rsa.pub >> "$RUNSWIFT_CHECKOUT_DIR"/image/home/nao/.ssh/authorized_keys
fi

# Set up flake8 and git hook linter, if flake8 is not installed.
# macOS should come with pip already installed, older Ubuntu versions may not.
myecho "Installing flake8 linter"
command -v pip >/dev/null 2>&1 || sudo apt-get install python-pip -y  # Ubuntu
command -v flake8 >/dev/null 2>&1 || sudo pip install flake8
if ! grep -q "runswift_test.sh" "$RUNSWIFT_CHECKOUT_DIR"/.git/hooks/pre-commit; then
  myecho "Installing git hook to catch coding style violations"
  mkdir -p "${RUNSWIFT_CHECKOUT_DIR}/.git/hooks/"
  echo "${RUNSWIFT_CHECKOUT_DIR}/bin/runswift_test.sh --python-files-only" >> "$RUNSWIFT_CHECKOUT_DIR"/.git/hooks/pre-commit
  chmod u+x "$RUNSWIFT_CHECKOUT_DIR"/.git/hooks/pre-commit
fi

########### Toolchain ##########

# CTC
mkdir -p "$RUNSWIFT_CHECKOUT_DIR"/ctc
cd "$RUNSWIFT_CHECKOUT_DIR"/ctc

export ASSETS_LOCATION="https://github.com/UNSWComputing/rUNSWift-assets/releases/download/v2017.1/"
export LINUX_CTC_ZIP=ctc-linux64-atom-2.1.4.13.zip
# TODO: Perhaps switch to Docker?
export OSX_CTC_ZIP=ctc-mac64-atom-2.1.3.3.zip
export BOOST_1550_LIBS=boost1550libs.zip
export BOOST_HEADERS=boostheaders.zip
export LIBUUID=libuuid.so.1.3.0

if [[ "$OSTYPE" == "linux-gnu" ]]; then
  if [ ! -f ${LINUX_CTC_ZIP} ]; then
    echo "Please provide the toolchain zip file: $LINUX_CTC_ZIP in $RUNSWIFT_CHECKOUT_DIR/ctc"
    # Aldebaran should provide a direct download link !!!    
    echo "Note: This can be found from Softbank/Aldeberan's documentation page."
  fi

  if [ ! -f ${BOOST_HEADERS} ]; then
    echo "Downloading modified boost headers"
    wget --continue --timestamping ${ASSETS_LOCATION}${BOOST_HEADERS}
  fi

  if [ ! -f ${BOOST_1550_LIBS} ];  then
    echo "Downloading pre-compiled boost 1.55.0 libs"
    wget --continue --timestamping ${ASSETS_LOCATION}${BOOST_1550_LIBS}
  fi

  if [ ! -f ${LIBUUID} ]; then
    echo "Downloading libuuid.so.1.3.0"
    wget --continue --timestamping ${ASSETS_LOCATION}${LIBUUID}
  fi

  if [ -f ${LINUX_CTC_ZIP} ]; then
    # Replace .zip with empty string
    export CTC_DIR="$RUNSWIFT_CHECKOUT_DIR"/ctc/${LINUX_CTC_ZIP/.zip/}
    [[ -d "$CTC_DIR" ]] || ( myecho Extracting cross toolchain, this may take a while... && unzip -q ${LINUX_CTC_ZIP} )
  fi

  if [ -f ${BOOST_HEADERS} ]; then
    export BOOST_HEADER_DIR="$RUNSWIFT_CHECKOUT_DIR"/ctc/${LINUX_CTC_ZIP/.zip/}/boost/include/boost-1_55/boost/type_traits/detail/
    unzip -j -q -o ${BOOST_HEADERS} -d ${BOOST_HEADER_DIR}
  fi

  if [ -f ${BOOST_1550_LIBS} ]; then
    export BOOST_1550_LIB_DIR="$RUNSWIFT_CHECKOUT_DIR"/ctc/boost_libs/
    unzip -j -q -o ${BOOST_1550_LIBS} -d ${BOOST_1550_LIB_DIR}
    # sudo unzip -j -q -o ${BOOST_1550_LIBS} -d /usr/lib/i386-linux-gnu/
  fi

elif [[ "$OSTYPE" == "darwin"* ]]; then
  if [ ! -f ${OSX_CTC_ZIP} ]; then
    echo "Please provide the toolchain zip file: $OSX_CTC_ZIP in $RUNSWIFT_CHECKOUT_DIR/ctc"
    # Aldebaran should provide a direct download link !!!
  fi
  # Mac OSX
  # Install homebrew, cmake, wget
  /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  brew install cmake || true
  brew install wget || true
  if [ -f ${OSX_CTC_ZIP} ]; then
    # Replace .zip with empty string
    export CTC_DIR="$RUNSWIFT_CHECKOUT_DIR"/ctc/${OSX_CTC_ZIP/.zip/}
    [[ -d "$CTC_DIR" ]] || ( myecho Extracting cross toolchain, this may take a while... && unzip -q ${OSX_CTC_ZIP} )
  fi
fi

if ! grep -q "CTC_DIR" ~/.bashrc ; then
echo export CTC_DIR=\"$CTC_DIR\" >> ~/.bashrc
fi

if [[ "$OSTYPE" == "linux-gnu" ]]; then
  echo "Changing permission on ctc dir"
  chmod -R 755 ${CTC_DIR}/cross/bin
  chmod -R 755 ${CTC_DIR}/cross/i686-aldebaran-linux-gnu/bin
  chmod -R 755 ${CTC_DIR}/cross/libexec/
fi

# Jayen's magic sauce
mkdir -p "$CTC_DIR"/../sysroot_legacy/usr/
myecho Downloading/extracting sysroot_legacy/usr, this may take a *long* time...
SYSROOT_ARCHIVE="sysroot_legacy.tar.gz"
if [ ! -f ${SYSROOT_ARCHIVE} ]; then
  wget --continue --timestamping ${ASSETS_LOCATION}${SYSROOT_ARCHIVE}
  tar -zxf ${SYSROOT_ARCHIVE}
fi

# Create nao symlink
myecho "Creating /home/nao symlink..."
if [ ! -d /home/nao ]; then
    sudo ln -s $RUNSWIFT_CHECKOUT_DIR/image/home/nao/ /home/nao
fi

# Create model symlink
myecho "Creating GMM symlink..."
if [ ! -f /home/nao/data/ball_classifier.gmm ]; then
    sudo ln -s $RUNSWIFT_CHECKOUT_DIR/ml_models/ball_classifier.gmm /home/nao/data/ball_classifier.gmm
fi
if [ ! -f /home/nao/data/ball_classifier.pca ]; then
    sudo ln -s $RUNSWIFT_CHECKOUT_DIR/ml_models/ball_classifier.pca /home/nao/data/ball_classifier.pca
fi
############ Building ###########

myecho Generating Makefiles and doing the initial build
echo

if [[ "$OSTYPE" == "linux-gnu" ]]; then
  export NPROC=$(nproc)
elif [[ "$OSTYPE" == "darwin"* ]]; then
  # Mac OSX
  export NPROC=$(sysctl -n hw.ncpu)
fi

# Build!
export TOOLCHAIN_FILE="$RUNSWIFT_CHECKOUT_DIR"/toolchain-2.1.cmake
for i in release relwithdebinfo; do
  cd "$RUNSWIFT_CHECKOUT_DIR"
  mkdir -p build-$i
  cd build-$i
  myecho $CTC_DIR
  cmake --debug-trycompile .. -DBoost_NO_BOOST_CMAKE=1 -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE} -DCMAKE_BUILD_TYPE=$i
  make -j${NPROC}
done

echo
echo All done! To build, type make -j${NPROC} in the build-release and build-relwithdebinfo directories.
echo

# Finish
echo Please close all shells.  Only new shells will have RUNSWIFT_CHECKOUT_DIR set to $RUNSWIFT_CHECKOUT_DIR
echo 'Alternatively, type . ~/.bashrc in existing shells.'
echo
