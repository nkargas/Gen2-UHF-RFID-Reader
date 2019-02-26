#! /bin/sh
cd gr-rfid
mkdir build
cd build
cmake ../
make
make test
sudo make install
sudo ldconfig
cd ../misc
mkdir data
cd data
touch source matched_filter gate decoder
cd ../../apps
