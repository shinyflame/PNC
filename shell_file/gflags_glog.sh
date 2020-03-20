set -e

cd "$(dirname "${BASH_SOURCE[0]}")"


# Install gflags.
wget https://github.com/gflags/gflags/archive/v2.2.0.tar.gz
tar xzf v2.2.0.tar.gz
mkdir gflags-2.2.0/build
pushd gflags-2.2.0/build
CXXFLAGS="-fPIC" cmake -DBUILD_SHARED_LIBS=true ..
sudo make -j8
sudo make install
popd

# Install glog which also depends on gflags.
wget https://github.com/google/glog/archive/v0.3.5.tar.gz
tar xzf v0.3.5.tar.gz
pushd glog-0.3.5

./configure --enable-shared

sudo make CXXFLAGS='-Wno-sign-compare -Wno-unused-local-typedefs -fPIC -D_START_GOOGLE_NAMESPACE_="namespace google {" -D_END_GOOGLE_NAMESPACE_="}" -DGOOGLE_NAMESPACE="google" -DHAVE_PTHREAD -DHAVE_SYS_UTSNAME_H -DHAVE_SYS_SYSCALL_H -DHAVE_SYS_TIME_H -DHAVE_STDINT_H -DHAVE_STRING_H -DHAVE_PREAD -DHAVE_FCNTL -DHAVE_SYS_TYPES_H -DHAVE_SYSLOG_H -DHAVE_LIB_GFLAGS -DHAVE_UNISTD_H'
sudo make install
popd

# Clean up.
#rm -fr /usr/local/lib/libglog.so*
#rm -fr v2.2.0.tar.gz gflags-2.2.0 v0.3.5.tar.gz glog-0.3.5
sudo rm -fr  gflags-2.2.0  glog-0.3.5bash

