#Exit if an error happen
set -e

#Get current dir and open
cd "$(dirname "${BASH_SOURCE[0]}")"

wget https://github.com/ApolloAuto/qp-oases/archive/v3.2.1-1.tar.gz
tar xzf v3.2.1-1.tar.gz

pushd qp-oases-3.2.1-1
mkdir bin
make -j8 CPPFLAGS="-Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion \
                   -Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE \
                   -D__NO_COPYRIGHT__"
sudo cp bin/libqpOASES.so /usr/local/lib
sudo cp -r include/* /usr/local/include
popd

# Clean up.
#rm -fr v3.2.1-1.tar.gz qp-oases-3.2.1-1
rm -fr  qp-oases-3.2.1-1
