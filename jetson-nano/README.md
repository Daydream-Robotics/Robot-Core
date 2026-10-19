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
copy the `jetson-nano` directory onto the Jetson Nano
cd jetson-nano
make clean && make
./MPCController





### If the OSQP library in external/osqp/lib/ doesn't work, rebuild it and replace the current library
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build
cd build
cmake ..
make -j
find build/out/libosqpstatic.a






