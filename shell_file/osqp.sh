#Exit if an error happen
set -e

#Get current dir and open
cd "$(dirname "${BASH_SOURCE[0]}")"

  wget https://github.com/ApolloAuto/osqp-contrib/archive/master.zip
  unzip master.zip
  pushd osqp-contrib-master
  mkdir -p /usr/local/include/osqp
  sudo cp -r osqp/include /usr/local/include/osqp/
  sudo cp osqp/libosqp.so /usr/local/lib/
  popd
