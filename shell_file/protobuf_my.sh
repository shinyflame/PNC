


#git clone https://github.com/protocolbuffers/protobuf.git

cd protobuf

git submodule update --init --recursive

./autogen.sh

./configure

sudo make
#make check
sudo make install
sudo ldconfig # refresh shared library cache.

