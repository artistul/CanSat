##################################
  
   Team: Volta Circuits  
   Author: Tonegari Stefan   

##################################

The code starts of with including all the necessary libraries and declaring variables. We then move on to defining all the pins according to a set of predefined rules:
  1) Separate every module with a comment
  2) Pin names begin with the module they belong to or their function
  3) Use the following abbreviations: (|| = or, x = number)
    -STBY = Standby  
    -PWMx = Pulse Width Modulation, number of the motor it's related to
    -ENx_A||B = Encoder, number of the motor it's related to, channel A or B
    -IMxA||B = In, Motor and the motors number, channel A or B
Then we have the additional declarations section for any other important variables or defines we may need. The next section is declaring all the modules.
In the setup function, we first initialize all the communications before moving onto initializing the modules. The MPU9250 has a calibration window during startup
for accurate measurements. If anything fails, it prints an error message and sends the code into an infinite loop.
In the loop function, all we do is call other functions.
void setupBQ(void) --> Used in the setup function, it sets up the BQ27441-G1A BMS.
void transmit(bool caser) --> Transmits the collected data through LoRa to the ground station. If caser is set true, it only sends the data for the main mission.
void getIMU() --> Updates the variables related to the MPU9250 module (3 axis linear acceleration, angular acceleration and magnetometer measurements).
void getBME() --> Updates the variables related to the BME680 module (temperature, humidity, pressure, VOC concentration and altitude).
void getGPS() --> Updates the variables related to the GPS module (latitude and longitude).
void getBQ() --> Updates the variables related to the BQ27441-G1A module (state of charge, battery voltage, average current draw, remaining capacity,
average power draw and state of health).
void rotate(bool axis, bool motor, int direction) --> Function used to control the motors, according to the following table:
  axis=1 -> steer | axis=0 -> incline
  direction = 1 -> forward | direction = -1 -> reverse | direction = 0 -> brake
  steer     motor 1 = right      (run M1)
  steer     motor 2 = left       (run M3)
  incline   motor 1 = speed up   (run M1 & M3)
  incline   motor 2 = brake      (run M2 & M4)
