# Setup GNU Radio with Python Virtual Environment

## Virtual Environment Setup
Create a new Python virtual environment
```
python3.11 -m venv ~/gnuradio
```
Edit the `~/gnuradio/bin/activate` shell script, add the following lines 
before the last statement.
```
CPATH="$VIRTUAL_ENV/"include
export CPATH
LIBRARY_PATH="$VIRTUAL_ENV/"lib64
export LIBRARY_PATH
LD_LIBRARY_PATH="$VIRTUAL_ENV/"lib64
export LD_LIBRARY_PATH
PKG_CONFIG_PATH="$VIRTUAL_ENV/"lib/pkgconfig":$PKG_CONFIG_PATH"
export PKG_CONFIG_PATH
PYBOMBS_PREFIX="$VIRTUAL_ENV/"
export PYBOMBS_PREFIX
CMAKE_INSTALL_PREFIX="$VIRTUAL_ENV/"
export CMAKE_INSTALL_PREFIX
```
Install additional packages for GNU Radio dependency.
```
source ~/gnuradio/bin/activate
pip install packaging pygobject pyqt5 pyqtgraph mako pyopengl \
  pygccxml jsonschema pyyaml numpy==1.26.4 scipy click-plugins
```

## Install GNU Radio
Download the release version (3.10.12) of GNU Radio and
extract the source files.
```
cd ~/Download/
wget https://github.com/gnuradio/gnuradio/archive/refs/tags/v3.10.12.0.tar.gz
tar xf v3.10.12.0.tar.gz
```
Activate the virtual environment before compiling and installing GNU Radio.
```
source ~/gnuradio/bin/activate
cd gnuradio-3.10.12.0
mkdir build
cd build
cmake .. -Wno-dev
make -j4
make install
```

## Install OOT Modules
Activate the virtual environment before compiling and installing out-of-tree
(OOT) modules.
```
source ~/gnuradio/bin/activate
cd <gr_oot_project>
mkdir build
cd build
cmake .. -Wno-dev
make -j4
make install
```

## Running GNU Radio
Activate virtual environment before running GNU Radio flowgraphs or 
the GNU Radio companion program (GRC).
```
source ~/gnuradio/bin/activate
gnuradio_companion
```




