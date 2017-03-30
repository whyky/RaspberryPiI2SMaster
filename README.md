# Raspberry Pi I2S Driver (Slave)

Low Level I2S character driver for the Raspberry Pi or Pi Zero where the Pi is configured as the I2S slave. In this case, the I2S clock and frame sync signals are provided by the codec. This avoids the issue of trying to generate the correct frequencies for audio with the Pi's oscillator.

The driver simply handles the I2S communication. Any additional functions such as audio file playback or recording is handled in userspace.

Note this was only tested on the Pi Zero, but should work on the original Raspberry Pi as well. Changing the base peripheral address in i2s_driver.h may allow it to run on a Pi 2 or Pi 3, but hasn't been tested.

Also make sure that the kernel source for your current kernel version is on your device.

## Installation

### Prerequisites
The setup for this driver depends on two external pieces of code to configure the GPIO pins.  One of which is from the Raspberry Pi forums and included in this repo. The other is WiringPi which should be installed by default. Check the [WiringPi] website for installation instructions if necessary.

<pre><code>#Make sure there is an entry in /lib/modules matching your kernel version
uname -r
ls /lib/modules

#Check if WiringPi is installed
gpio -v
</pre></code>

Compile TimG's GPIO wrapper from the Raspberry Pi forums:

In the directory where this repo has been cloned:
<pre><code>gcc -o gpio_alt gpio_alt.c
sudo chown root:root gpio_alt
sudo chmod u+s gpio_alt
sudo mv gpio_alt /usr/local/bin/
</pre></code>

Now we can set GPIO pins to their alternative functions through the command line. 

### Install Scripts

The next step requires that you have the kernel source tree on your device in order to complile the code as a loadable kernel module. If necessary, modify the path to your kernel source in the Makefile located in the i2s_driver_src directory. 

The i2s-install script will build and load the module and will also set the corresponding GPIO pins to their I2S functions.  
<pre><code>cd i2s_driver_src
./i2s-install.sh
</pre></code>

If you just want to compile the .ko and do the rest your self just use 
<pre><code>cd i2s_driver_src
sudo make
</pre></code>
Then you can use modprobe to load the module and a method of your choice to configure the GPIO.

To remove the driver:
<pre><code>./i2s-uninstall.sh
</pre></code>
This will remove the module, clean the directory, and set the GPIO pins back to their default functions with WiringPi.

The included example program can be compiled and run by:
<pre><code>cd driver_test_src
sudo make
sudo ./driver_test trem_sine.wav
</pre></code>
There are two #define's at the beginning of driver_test.c that can be used to enable or disable the demos for the TX or RX portion. If DO_TX is enabled, a 24-bit 48kHz wav file is required. A modulated sine wave made in Audacity is provided in the directory.


## Usage
This driver supports the basic file functions and ioctl.  If using C, <unistd.h> should be included.

Any read and write commands should use a multiple of 4 for the size due to the samples being stored as int32_t.

<pre><code>#include unistd.h
int i2s = open("/dev/i2s, O_RDWR);

int space_remaining = ioctl(i2s, I2S_TX_BUFF_SPACE);
int samples_available = ioctl(i2s, I2S_RX_BUFF_ITEMS);

write(i2s, transmit_data_buffer, 4*space_remaining);    // Where transmit_data_buffer is a pointer to a block of data to be sent
read(i2s, receive_data_buffer, 4*samples_available);    // Where receive_data_buffer is a pointer to wherever the data is to be stored

close(i2s);
</code></pre>

## IOCTL Commands
The following commands can be used with ioctl to modify things during run-time.  Any of the write commands (numbers 7-16) must be preceeded by an ioctl(<file>, I2S_SET_EN, 0) to disable the interface. 

For additional information on the registers, see the [BCM2835 ARM Peripherals] data sheet.

0. I2S_SET_EN - Enable or disable the interface. Write either a 1 or 0.

1. I2S_SET_TXON - Enable or disable the transmitter.  Write either 1 or 0.

2. I2S_SET_RXON - Enable or disable the receiver. Write either 1 or 0.

3. I2S_TX_BUFF_SPACE - Returns the number of empty spaces in the kernel's transmit buffer

4. I2S_RX_BUFF_ITEMS - Returns the number of samples available to be read in the kernel's receive buffer

5. I2S_CLEAR_TX_BUFF - Reset the kernel's transmit buffer

6. I2S_CLEAR_RX_BUFF - Reset the kernel's receive buffer

7. I2S_WRITE_CS_A - Write the given value to the CS_A register.

8. I2S_WRITE_MODE_A - Write the given value to the MODE_A register.

9. I2S_WRITE_RXC_A - Write the given value to the RXC_A register.

10. I2S_WRITE_TXC_A - Write the given value to the TXC_A register.

11. I2S_WRITE_DREQ_A - Write the given value to the DREQ_A register.

12. I2S_WRITE_INTEN_A - Write the given value to the INTEN_A register.

13. I2S_WRITE_INTSTC_A - Write the given value to the INTSTC_A register

14. I2S_WRITE_GRAY - Write the given value to the GRAY register.

15. I2S_CLR_TX_FIFO - Clear hardware TX FIFO. Requires up to two I2S clocks to take effect.

16. I2S_CLR_RX_FIFO - Clear hardware RX FIFO. Requires up to two I2S clocks to take effect.

## Codec Interface
Any code for interacting with a codec or other external device is done in userspace. There is a Python script in the root directory that was written for the WM8731 codec. It simply uses the SMBus library to set up the codec over I2C.  The functions are laid out like the registers in the data sheet and should be self explanatory.

## Acknowledgements
Thanks to TimG on the [Raspberry Pi forums] for his GPIO wrapper.

Thanks [Philpoole] for the inpsiration to write my own driver.

Software FIFO code based on Stratify Labs [example].

[Raspberry Pi forums]: <https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=39138>
[example]: <https://stratifylabs.co/embedded%20design%20tips/2013/10/02/Tips-A-FIFO-Buffer-Implementation/>
[Philpoole]: <https://github.com/philpoole/>
[BCM2835 ARM Peripherals]: <https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf>
[WiringPi]: <http://wiringpi.com/download-and-install/>
