Contents:

* Sources files of:
* RTDM kernel driver for GPMC
* Normal kernel driver for GPMC
* RTDM userspace driver for GPMC
* RaMstix userspace driver to control hardware components.

For the haptic pedals the Makefile is altered to work without XENOMAI. In order to write software for the RaMstix, make a new file in /src/ramstix/unit-tests and call it *_test.c. Then from this folder run '''make all''' which builds the tests into the build folder for the RaMstix to run. 

This is not the optimal way to work with the RaMstix, for additional information and help ask Marcel Schwirtz. 
