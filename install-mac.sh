#! /bin/bash
brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig
cd build
OPENSSL_VERSION=`brew list --versions openssl | cut -d' ' -f2`
#! OPENSSL_VERSION="1.1.1d"
brew install openssl libuv cmake
#! cmake -DOPENSSL_ROOT_DIR=$(brew --cellar openssl)/$OPENSSL_VERSION -DOPENSSL_LIBRARIES=$(brew --cellar openssl)/$OPENSSL_VERSION/lib ..
cmake -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl -DOPENSSL_LIBRARIES=/usr/local/opt/openssl/lib ..
#! cmake -DOPENSSL_ROOT_DIR=$(brew --prefix openssl) -DOPENSSL_LIBRARIES=$(brew --prefix openssl)/lib ..
make
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
