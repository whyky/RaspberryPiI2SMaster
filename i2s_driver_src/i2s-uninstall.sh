sudo rmmod i2s_driver
if [[ -e /dev/i2s ]] ; then
	sudo rm /dev/i2s
fi
if [[ -e i2s_driver.c ]] ; then
	sudo make clean
fi
gpio -g mode 18 in
gpio -g mode 19 in
gpio -g mode 20 in
gpio -g mode 21 in

