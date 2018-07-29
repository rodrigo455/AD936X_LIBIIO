# AD936X LIBIIO REDHAWK DEVICE
 
## Description

Note: This project is being updated. software decimation and interpolation was a featured taken from [gr-iio](https://github.com/analogdevicesinc/gr-iio) and it was removed by analogdevices, since it is indeed incorrect, because no filtering is applied.

The project is based in [REDHAWK USRP_UHD](https://github.com/RedhawkSDR/USRP_UHD) 
and it contains the source and build script for the REDHAWK Device AD936X_LIBIIO
device. This device is a FRONTEND Interfaces compliant device for the AD936x that
requires the [libiio](https://github.com/analogdevicesinc/libiio) and 
[libad9361-iio](https://github.com/analogdevicesinc/libad9361-iio) libraries to be installed.

## Installation Instructions

This project requires the libiio and libad9361 libraries, which must be installed in order to build and run this asset.

### libiio

Follow Analog Devices' [What is libiio?](https://wiki.analog.com/resources/tools-software/linux-software/libiio#building_on_the_linux_host_target) wiki page, in order to build libiio for Debian-flavoured GNU/Linux distributions.

For CentOs you're able to get the required dependencies with the command:
```
$ sudo yum install libxml2 libxml2-devel libaio-devel libusbx-devel avahi-devel
```
then you can fetch the source, build and install
```
$ git clone https://github.com/analogdevicesinc/libiio.git
$ cd libiio
$ cmake ./
$ make all
$ sudo make install
```
### libad9361-iio

Just fetch the source, build and install
```
$ git clone https://github.com/analogdevicesinc/libad9361-iio.git
$ cd libad9361-iio
$ cmake ./
$ make all
$ sudo make install
```
### AD936X_LIBIIO

You can import this project to your workspace using the REDHAWK Import Wizard, 
build and install to $SDRROOT within the REDHAWK IDE. Otherwise run the `build.sh` script found at the
top level directory. To install to $SDRROOT, run `build.sh install`. Note: root
privileges (`sudo`) may be required to install.

## Copyrights

This work is protected by Copyright. Please refer to the
[Copyright File](COPYRIGHT) for updated copyright information.

## License

AD936X LIBIIO REDHAWK DEVICE is licensed under GNU Lesser General Public License (LGPL).

## Troubleshooting

* if building libiio, you got: `Check size of struct usb_functionfs_descs_head_v2 - failed`, use the following commands to fix it, being able to support usb backend.
```
$ wget https://raw.githubusercontent.com/torvalds/linux/master/include/uapi/linux/usb/functionfs.h
$ sudo mv functionfs.h /usr/include/linux/usb/functionfs.h
```
