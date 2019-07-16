#git pull origin master
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
