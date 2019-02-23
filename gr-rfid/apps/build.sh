#! /bin/sh
git pull origin 190221_patch
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
