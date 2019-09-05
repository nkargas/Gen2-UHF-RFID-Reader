git pull origin preview_1.3.0
cd ../build
cmake ../
make
make test
make install
ldconfig
cd ../apps
