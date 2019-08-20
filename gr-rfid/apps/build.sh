#git pull origin master
cd ../build
cmake ../
make
make test
make install
ldconfig
cd ../apps
