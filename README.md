Sous-Viduino
============
Sous-vide powered by Arduino - The SousViduino!

This project was designed as an alternative to the [Adafruit Sous-Vide controller](http://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino) by Bill Earl, but with the following changes:

* Instead of the Adafruit RGB LCD Shield, it uses the cheaper DFRobot Keypad Shield
* Startup state is RUN instead of OFF, to recover from power outages. Turn it off with a physical switch when not in use!
* The SELECT button toggles between various increments when changing setpoint and PID values
* The UP button triggers autotune
* Splash screen delay reduced
* On startup, waits until it can read the temperature sensor
* Configurable Active Low or Active High Relay output
* Configurable DFRobot Keypad version (V1.0 or V1.1)


Instructions
------------

###Build

1. Download.
2. Install Required 3rd Party Libraries:
  * [Arduino PID Library](http://playground.arduino.cc/Code/PIDLibrary) - Can also install this from the built-in library manager
  * [Arduino AutoTune Library](http://playground.arduino.cc//Code/PIDAutotuneLibrary)
3. Assemble hardware according to the [Adafruit Sous-Vide controller tutorial](http://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino).
4. Build and Deploy.
 
###Usage

TODO


References
----------

Based on the Sous Vide Controller by Bill Earl:  
https://github.com/adafruit/Sous_Viduino

Which is in turn based on the Arduino PID and PID AutoTune Libraries by Brett Beauregard:  
http://playground.arduino.cc/Code/PIDLibrary  
http://playground.arduino.cc//Code/PIDAutotuneLibrary

