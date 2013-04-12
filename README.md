arducotper
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
- Control over pitch through RC
- Controll over vertical speed through RC
- Calculation of current pitch, roll and yaw via the DMP unit on the MPU-6050

Planned features are:

- Automatic stabilisation of flight via MPU-6050
- Control of roll and yaw through RC
- Assisted safe landing through free fall detection


More features will be added as I expand the usage of my quadcopter.





Modifying the sketch
====================

As I mentioned earlier, the sletch relies on certain hardware, but should work with all similar hardware aswell.
Fundemtal paramters of the used hardware, like the pin it's connected to, or minimum/maximum values of the ESCs are #define 'd
at the top of the sketch and can easily be changed to whatever value you prefer.

You might also want to change how the motor speed is affected by the signal coming in from the RC, for exmaple if your
copter is lighter or heavier than mine. At the moment this can only be done by changing the used equations in
``calculateVelocities()``.
However, I will improve my algorithms in the future and will probably find a way to easily fine tune the paramters of
the motor control.

Note:

The MPU-6050 board currently is the only supported gyro/accelerometer hardware and can only be connected in one specific 
way, due to the requirements of I2C and the fixed position of the Arduino's external interrupt pin.


If you have any questions, feel free to contact me, however I cannot guarantee that this sketch will work for 
everyone.
