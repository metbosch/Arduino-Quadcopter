Arduino-Quadcopter
=========

An arduino sketch for controlling a quadcopter. Uses RC and the MPU-6050 chip.


This is a simple arduino sketch that allows you to take over control over a quadcopter with the arduino board.
Because I've made this sketch primarily for private use, the sketch relies on certain hardware, but should also be
usable with different hardware with a bit of customization work.

This sketch was only tested on the Arduino Uno rev 3 board, so it might not work on other arduino-like boards without
making changes.

The arducopter sketch needs three things to work at the moment:

- A RC receiver with a minimum of 4 channels
- The MPU-6050 gyro/accelerometer chip
- Four ESCs with a known minimum and maximum value


At the moment, the sketch doesn't implement all features as I can only update it as fast as I can run new tests
with my quadcopter.
Complete features are:

- Arming of ESCs with given min/max values
- Control over pitch, roll and yaw through motor speed balances
- Control over pitch, roll and yaw through RC
- Controll over vertical speed through RC
- Calculation of current pitch, roll and yaw via the DMP unit on the MPU-6050

Planned features are:

- Automatic stabilisation of flight via MPU-6050 (complete but not fine tuned)
- Assisted safe landing through free fall detection


More features will be added as I expand the usage of my quadcopter.


In order to use this sketch, you will need to install the libraries from [arducopter/libraries](https://github.com/strangedev/arducopter/tree/master/libraries)
on your computer.

Note:
You cannot use the original Arduino-PID-Library wih this sketch. Please use the modified one included in this repo.

I want to thank [Jeff Rowberg](https://github.com/jrowberg), who created the great library [i2cdevlib](https://github.com/jrowberg/i2cdevlib) for using all
kinds of I2C devices including the MPU-6050 with the arduino board.
I also want to thank [br3ttb](https://github.com/br3ttb), who created the [Arduino-PID-Libary](https://github.com/br3ttb/Arduino-PID-Library/) which helped me save a lot of time.
Without their contributions to the open source community I couldn't have done this project.


Modifying the sketch
====================
***
*Note:* If you have questions about the code or the compatability of the code and certain hardware, you've spotted a bug or need help with your project, please open a new github issue rather than sending me an e-mail. That way, I don't have to answer questions twice and you'll get quicker answers. Thank you.
***
As I mentioned earlier, the sketch relies on certain hardware, but should work with all similar hardware aswell.
Fundemtal paramters of the used hardware, like the pin it's connected to, or minimum/maximum values of the ESCs are ``#define``'d
at the top of the sketch and can easily be changed to whatever value you prefer.

You might also want to change how the motor speed is affected by the signal coming in from the RC, for exmaple if your
copter is lighter or heavier than mine. At the moment this can only be done by changing the used equations in
``calculateVelocities()``.
However, I will improve my algorithms in the future and will probably find a way to easily fine tune the paramters of
the motor control.

Note:

The MPU-6050 board currently is the only supported gyro/accelerometer hardware and can only be connected in one specific 
way, due to the requirements of I2C and the fixed position of the Arduino's external interrupt pin.

License
=======

This project is licensed under the MIT license (see the included license file for more information).

Note: The libraries used in this project might not be licensed under the MIT license. I've made sure that the usage of all libraries as a part of this project, my modifications and the redistribution of the modified and unmodified libraries are permitted by the respective licenses, but the permissions of the MIT license might not apply, especially in regard to relicensing. 
