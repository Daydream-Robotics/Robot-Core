## At start (Installs):
sudo apt update
sudo apt upgrade -y
sudo apt install -y \
  python3-pip \
  python3-serial \
  git \
  cmake \
  build-essential \
  htop \
  screen \
  minicom


## Set-up Serial:
sudo usermod -a -G dialout $USER
stty -F /dev/ttyACM1 raw -echo -onlcr
sudo reboot


## Make and run program
copy the microcontroller directory onto the microcontroller
cd microcontroller
make clean && make
./MPCController





### If the OSQP library in firmware/ doesn't work run this and replace the curr with the new one
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build
cd build
cmake ..
make -j
find build/out/libosqpstatic.a






