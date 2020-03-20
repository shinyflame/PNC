#Exit if an error happen
set -e

#Get current dir and open
cd "$(dirname "${BASH_SOURCE[0]}")"

wget https://github.com/google/protobuf/releases/download/v3.6.0/protobuf-cpp-3.6.0.tar.gz
tar xzf protobuf-cpp-3.6.0.tar.gz

pushd protobuf-3.6.0
./configure --prefix=/usr/local/protobuf
sudo make -j8
sudo make check
sudo make install
popd

# Clean up.
#rm -fr protobuf-cpp-3.3.0.tar.gz protobuf-3.3.0
rm -fr  protobuf-3.6.0
