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

# apt's cmake on the Nano (Ubuntu 18.04) is 3.10; OSQP needs >= 3.18
pip3 install --user cmake
# check with: cmake --version   (if it still says 3.10, add ~/.local/bin to PATH)


## Set-up Serial:
sudo usermod -a -G dialout $USER
stty -F /dev/ttyACM1 raw -echo -onlcr
sudo reboot


## Get OSQP source (once per clone)
OSQP v1.0.0 and QDLDL v0.1.8 are pinned git submodules in external/ (Eigen is vendored in external/eigen).
git submodule update --init --recursive
(if copying onto the Nano instead of cloning, run this first, then copy both jetson-nano/ and shared/ keeping them side by side)


## Build and run program
cd jetson-nano
cmake -B build
cmake --build build -j4
./build/MPCController

Only rebuild after code changes; the program stays in build/.
For a clean rebuild: rm -rf build






