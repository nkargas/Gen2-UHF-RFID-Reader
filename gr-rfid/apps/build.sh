git pull origin preview_1.3.0
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
