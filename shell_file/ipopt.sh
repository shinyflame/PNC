set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

sudo add-apt-repository universe && sudo apt-get -y update
sudo apt-get install -y libblas-dev liblapack-dev gfortran

wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.11.zip -O Ipopt-3.12.11.zip
unzip Ipopt-3.12.11.zip

pushd Ipopt-3.12.11/ThirdParty/Mumps
bash get.Mumps
popd

pushd Ipopt-3.12.11
./configure --build=x86_64 --disable-shared ADD_CXXFLAGS="-fPIC" ADD_CFLAGS="-fPIC" ADD_FFLAGS="-fPIC"
sudo make -j8 all
sudo make install
mkdir -p /usr/local/ipopt
cp -r include /usr/local/ipopt/ && cp -r lib /usr/local/ipopt/
popd

# Clean up.
apt-get clean && rm -rf /var/lib/apt/lists/*
#rm -fr Ipopt-3.12.11.zip Ipopt-3.12.11
rm -fr  Ipopt-3.12.11
