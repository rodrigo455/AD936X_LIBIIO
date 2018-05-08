# AD936X LIBIIO REDHAWK DEVICE
 
## Description

Note: This work must be updated. software decimation and interpolation was a featured taken from [gr-iio](https://github.com/analogdevicesinc/gr-iio) and it was removed by analogdevices, since it is indeed incorrect, because no filtering is applied. Pluto makes decimation and interpolation available in hardware that this software does not support for now.

The project is based in [REDHAWK USRP_UHD](https://github.com/RedhawkSDR/USRP_UHD) 
and it contains the source and build script for the REDHAWK Device AD936X_LIBIIO
device. This device is a FRONTEND Interfaces compliant device for the AD936x that
requires the [libiio](https://github.com/analogdevicesinc/libiio) and 
[libad9361-iio](https://github.com/analogdevicesinc/libad9361-iio) libraries to be installed.

## Installation Instructions

This asset requires the libiio and libad9361 libraries, which must be installed in order to build
and run this asset. To build from source, run the `build.sh` script found at the
top level directory. To install to $SDRROOT, run `build.sh install`. Note: root
privileges (`sudo`) may be required to install.

## Copyrights

This work is protected by rh.USRP_UHD's Copyright. Please refer to the
[Copyright File](COPYRIGHT) for updated copyright information.

## License

AD936X LIBIIO REDHAWK DEVICE is licensed under rh.USRP_UHD's 
GNU Lesser General Public License (LGPL).
