# Progress

All the changes to electronics have been written in detail below along with the reasoning for each change made.

## Version 1.0

The Spiderbot hardware was assembled for the first time during Quark 2022 display. It was before 2021 batch induction was held. 

- The circuit consisted of a 11.1V LiPo battery which was stepped down by the a LM2596 [buck converter](/electronics/docs/power_management/voltage_converters.md) to 8.4V to power the motors. 
- The motors used were RKI-1206 16 kg-cm [servo motors](/electronics/docs/motors/servo_motor.md). 
- In total, Spiderbot had 18 servo motors (3 for each leg). 
- The motors were controlled by 2 [Arduino](/electronics/docs/microcontrollers/arduino.md) Mega R3 boards (each board controlling 3 legs or 9 motors). 
- The Arduino was given angles by the Raspberry Pi 4 Model B which was used as the onboard computer. The Raspberry Pi is powered by a powerbank.
- The motor connections were done by hand soldering on a Perfboard.

### Circuit Block Diagram

### Problems

- The LM2596 provides limited output current. 
- Current drawn by servo motors is proportional to the load. Not sure if any load or current calculations were done.
- A serparate powerbank is being used for the raspberry pi which adds to the weight.

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

During the semester break, our task was to design a [PCB](/electronics/docs/pcb) since resoldering perfboard was tedious. After this, the Spiderbot was able to stand and move with more stability.

### Circuit Schematic and Layout

### Problems

- We still have the common buck converter output problem.
- We don't know how much current is being drawn in circuit.
- We are still using a separate powerbank for the Raspberry Pi.
- The wiring is a mess.
- The PCB is too big to fit inside.
- The wired connections are unreliable.
- The motors don't seem to be getting enough power.

## Version 2.0

All the wiring is done between battery and the buck converter. So if we eliminate the buck converter from the circuit then we can eliminate all the wiring we have.

The buck converter is used to step down the battery voltage to 8.4V for the motors. The voltage rating for the motors is 6-8.4V. So we can just use a 7.4V battery.

The maximum area in the PCB is taken by the Arduino board. In order to reduce this area, we decided to use embedded microcontroller ic. We chose the [STM32F103C8T6](/electronics/docs/microcontroller/stm32.md) microcontroller which is used in the BluePill development board. We used 6 of these ICs (1 for each leg) to implement distributed control.

We designed a 4 layer PCB with the following stackup:
- Power/Signal
- GND
- 3.3V
- Signal

We also switch from Rosserial to [I2C](/electronics/docs/communication/i2c.md) to send angles from Raspberry Pi to STM32.

### Circuit Schematic and Layout

### Problems