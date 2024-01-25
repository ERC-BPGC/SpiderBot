# Progress

All the changes to electronics have been written in detail below along with the reasoning for each change made.

## Version 1.0

The Spiderbot hardware was assembled for the first time during Quark 2022 display. It was before 2021 batch induction was held. 

- The circuit consisted of a 11.1V LiPo battery which was stepped down by the a LM2596 [buck converter](/electronics/docs/power_management/voltage_converters.md) to 8.4V to power the motors. 
- The motors used were RKI-1206 16 kg-cm [servo motors](/electronics/docs/motors/servo_motor.md). 
- In total, Spiderbot had 18 servo motors (3 for each leg). 
- The motors were controlled by 2 [Arduino](/electronics/docs/microcontrollers/arduino.md) Mega R3 boards (each board controlling 3 legs or 9 motors). 
- The Arduino was given angles by the Raspberry Pi 4 Model B which was used as the onboard computer.
- The motor connections were done by hand soldering on a Perfboard.

### Problems

- The LM2596 provides limited output current. 
- Current drawn by servo motors is proportional to the load.
- Not sure if any load or current calculations were done.

### Consequence

- The servo motors tried to pull more current than the buck converter was rated for. As a result, the buck converter and some of the servo motors heated up and burned.

### Datasheets / Links
- LM2596: https://robu.in/product/lm2596s-dc-dc-buck-converter-power-supply/
- RKI 1206: https://robokits.co.in/motors/rc-servo-motor/metal-gear-dual-shaft-16kgcm-digital-servo-motor

## Version 1.1


