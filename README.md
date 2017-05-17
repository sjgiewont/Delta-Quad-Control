# Delta-Quad Control

Contains all code running on BeagleBone Black (or similar single board computer) to control the Delta-Quad.

 * Custom library for evaulating ANFIS
 	* Includes membership fucntions
 * inverseKinematics - evaluates the inverse kinematics of a single Delta-Leg using ANFIS
 * motion - some functions to move between two points
 * piecewiseMotion - generates various trajectories 
 * servoCalibrate - a easy to use script to help calibrate the servos
 * socket_test - uses sockets to stream commands from external PC
 * Blynk - python API for Blynk
 	* blynk_test - shows example of how to use Blynk API for Python
 * main - final walking control of Delta-Quad

##### Pickle Files
There are various .pkl files included in the repository. They are trained FIS structures that can be used to evaulate the Delta-Legs Kinematics

## Install 

BeagleBone Black

* Install Adafruit libraries to easily interface with BBB UART: [Adafruit_BBIO](https://learn.adafruit.com/setting-up-io-python-library-on-beaglebone-black/installation-on-ubuntu)

ANFIS

* NumPy
* SciPy
* [scikit-fuzzy](https://github.com/scikit-fuzzy/scikit-fuzzy)
* [anfis](https://github.com/twmeggs/anfis)
