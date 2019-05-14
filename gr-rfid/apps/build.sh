git pull origin 1904_renewal
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
