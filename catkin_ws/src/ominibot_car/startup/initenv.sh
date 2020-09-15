#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="0403", MODE:="6015", GROUP:="dialout",  SYMLINK+="ominibot_car"' >/etc/udev/rules.d/ominibot_car.rules

service udev reload
sleep 2
service udev restart

