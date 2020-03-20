#Exit if an error happen
set -e

#Get current dir and open

sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo mkdir build
cd build
sudo cmake ..
sudo make
sudo cp libgtest*.a /usr/local/lib
#popd
#popd

#Clean

