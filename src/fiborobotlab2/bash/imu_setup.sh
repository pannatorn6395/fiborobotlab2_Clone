cd ~
git clone https://github.com/RTIMULib/RTIMULib2.git
sed -i "s/spidev%d.%d/spidev1.0/g" ~/RTIMULib2/RTIMULib/RTIMUHal.cpp
cd ~/RTIMULib2/Linux/python
python3 setup.py build
sudo python3 setup.py install