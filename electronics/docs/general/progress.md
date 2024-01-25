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

### Circuit Block Diagram

### Problems

- The LM2596 provides limited output current. 
- Current drawn by servo motors is proportional to the load. Not sure if any load or current calculations were done.

### Consequence

- The servo motors tried to pull more current than the buck converter was rated for. As a result, the buck converter and some of the servo motors heated up and burned.

### Datasheets / Links
- LM2596: https://robu.in/product/lm2596s-dc-dc-buck-converter-power-supply/
- RKI 1206: https://robokits.co.in/motors/rc-servo-motor/metal-gear-dual-shaft-16kgcm-digital-servo-motor

## Version 1.1

At this time, the 2021 batch was inducted. Our first task in Spiderbot was to go through circuit connections, draw the block diagram and do calculations. 

- The load problem was already identified and the RKI-1206 on joint 2 or each leg were replaced with new RKI-1202 35 kg-cm servo motors.
- We calculated the worst case current and suggested a new buck converter to be used.

### Calculations

All motors were connected in parallel so the total current through the buck converter will be:

$$ I_{total} = I_1 + I_2 + ... + I_{18}$$

where $I_n$ is the current through a single servo motor.

In the worst case scenario, all servo motors will draw the maximum stall current, which by the datasheet is **3.5A**.

$$ I_{total,max} = 18 \times 3.5 = 63A $$

However, this is possible only if all servo motors are carrying maximum load, which according to our design, is not possible. Most of the current will be drawn by the servo motors at joint 2 while joint 1 and joint 3 will hardly draw any current. Therefore, we intuitively decided that **20A** current should be enough.

### Modifications

- The battery was now connected to 2 10A buck converters in parallel.
- The outputs of the buck converters were then connected together into a single overall output.
- Motors at Joint 2 of all legs were replaced by RKI-1202 servo motors.

### Circuit Block Diagram

### Problems

- We now had issues with connections on the perfboard which we had to solder repeatedly.
- We did not consider this at the time but connecting the outputs two buck converters to each other is not good in the long run due to output ripples (see [Buck Converter](/electronics/docs/power_management/voltage_converters.md)).

### Datasheets / Links

- Buck converter: https://www.electronicscomp.com/300w-10a-dc-dc-step-down-buck-converter-adjustable-constant-voltage-module?gad_source=1&gclid=Cj0KCQiAqsitBhDlARIsAGMR1Ri9iiJ6AeSPqgXqTcEVZz8vzKFG0npAefIj5UGB_D07DExkVCJvSTAaAkiNEALw_wcB
- RKI-1202: https://robokits.co.in/motors/rc-servo-motor/ultra-torque-dual-shaft-metal-gear-35kgcm-coreless-servo-w-t-acc

## Version 1.2
