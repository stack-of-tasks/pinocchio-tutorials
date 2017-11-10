#! /bin/sh

### Install basic dependencies 
sudo apt-get install -y cmake cmake-curses-gui cmake-qt-gui curl doxygen g++ ipython libassimp-dev assimp-utils libboost-dev omniidl-python omniorb omniorb-idl omniorb-nameserver libomniorb4-dev libomnithread3-dev python python-matplotlib python-numpy python-scipy python2.7 qgit libbz2-dev zlib1g-dev libssl-dev pax tar libeigen3-dev libtinyxml-dev liburdfdom-dev libboost-all-dev libpcre3-dev libopenscenegraph-dev libqt4-dev python-qt4-dev libncurses-dev python-pip cython python-h5py
echo 'export ROBOTPKG_BASE=/opt/openrobots' >> .bashrc
echo 'export PKG_CONFIG_PATH=$ROBOTPKG_BASE/lib/pkgconfig:${PKG_CONFIG_PATH}' >> .bashrc
echo 'export LD_LIBRARY_PATH=$ROBOTPKG_BASE/lib/:$ROBOTPKG_BASE/lib64/:${LD_LIBRARY_PATH}' >> .bashrc
echo 'export PATH=$PATH:$ROBOTPKG_BASE/sbin:$ROBOTPKG_BASE/bin' >> .bashrc
echo 'export PYTHON_LOCAL_PATH=lib/python2.7/dist-packages' >> .bashrc
echo 'export PYTHON_SITE_PATH=lib/python2.7/site-packages' >> .bashrc
echo 'export PYTHONPATH=${PYTHONPATH}:${ROBOTPKG_BASE}/${PYTHON_LOCAL_PATH}:${ROBOTPKG_BASE}/${PYTHON_SITE_PATH} ' >> .bashrc

### Setup RobotPKG repository
relcode=`lsb_release -cs`
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $relcode robotpkg
EOF

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt-get update

### Install Pinocchio
sudo apt-get install -y robotpkg-pinocchio robotpkg-osg-dae robotpkg-gepetto-viewer-corba

### Install other requirement (quadprog, tensorflow)
sudo chmod -R a+w /usr/local/
sudo pip install quadprog
export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-1.4.0-cp27-none-linux_x86_64.whl
pip install --upgrade $TF_BINARY_URL
pip install tflearn


