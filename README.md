# Changes in this fork
First of all, thanks to LinusB, RichA777 and hen3drik

## Firmware: 
* added debugging functions to make initial testing easier 
* added retry function if color is not fouond immediately. 
* average colors when learinig new color
* optimized stepper movement to allways take the shortest path
* replaced some (not all yet) magic numbers in the code by defines.
* some code restructuring

## Hardware
* added variant for stepper with 3mm sqaure shaft
* added variant of the Sequencer_tube: the original modified version sometimes got beads stuck in it.

## more Ideas:
 * aditional LED to color detection
 * color sorting by measuring the distance
 * plot color cloud (python script or something)
 * 2 stage analyzer: to parallelize detection and sorting.
   * replace delays with timeouts. 
   * change analyzer Stage

# original repo from @HEN3DRIK
- 👋 Hi, I’m @HEN3DRIK
- 👀 I’m interested in 3DPrinting and Electroforming/Electroplating
- 📫 How to reach me 
  - Youtube https://youtube.com/@hen3drik
  - Twitter https://twitter.com/hen3drik

# BeadSorter
Nice to have you here. In this repository I keep the files that are necessary for the BeadSorter. Also there will be documentation about the needed electronics and software. I will update this repo continuously.

# Video
[![perlerbead sorting machine](https://img.youtube.com/vi/CX-w85ZC5AQ/0.jpg)](https://www.youtube.com/watch?v=CX-w85ZC5AQ)

# References
These are Projects necessary for this beadsorter to work.

Original Project by Linus Barth:
[BeadSort: Machine for sorting perler beads by LinusB - Thingiverse](https://www.thingiverse.com/thing:2598302)

Modified to skip aligner phase by RichA777
[Bead Sorter modified to skip aligner phase by RichA777 - Thingiverse](https://www.thingiverse.com/thing:4507571)
# Parts List
You'll need electronic parts for this bead sorter. This is alist of what i used in this project. I belive some components can be substituted.

- 1x Miniature-DC-Gear Motor 15RPM: https://www.amazon.de/gp/product/B08591W5N8/ref=ppx_yo_dt_b_asin_title_o08_s00?ie=UTF8&psc=1
- 2x LM2596S DC-DC: https://www.amazon.de/gp/product/B07YWLCTLK/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1
- 1x A4988 Stepper Driver Module: https://www.amazon.de/gp/product/B07C2V9GWC/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1
- 1x Nema 17 Stepper Bipolar 1A 16Ncm 42x20mm 4-Wires: https://www.amazon.de/dp/B06XVM38YW/ref=pe_27091401_487024491_TE_item
- 1x L298N Motor Drive Controller Board Module: https://www.amazon.de/gp/product/B07MY33PC9/ref=ppx_yo_dt_b_asin_title_o08_s01?ie=UTF8&psc=1
- 1x Adafruit-Sensor TCS34725 RGB: https://www.amazon.de/gp/product/B00OKCRU5M/ref=ppx_yo_dt_b_asin_title_o09_s01?ie=UTF8&psc=1
- 1x Arduino Nano V3.0 Atmega328 CH340: https://www.amazon.de/gp/product/B01MS7DUEM/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1
- 1x SG90 Micro Servo Motor 9G: [https://www.amazon.de/gp/product/B07CYZSVKW/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1](https://www.amazon.de/owootecc-Helicopter-Airplane-Remote-Controls/dp/B07RZK4K39/ref=sr_1_7?keywords=sg90+servo&qid=1692117042&sprefix=sg90%2Caps%2C88&sr=8-7)
- 1x 100µF Capacitor
- 1x Resistor matching LED
- 1x White LED
- 1x Photo Resistor 5mm
- 1x Push Button
- Wires
- PTFE Spray

# Electronics Schema
I know, this isn't nice looking.
This whole thing is powered by a 12V power supply. I used a 9VA; That thing doesnt requiere a lot of current.
![alt text](https://github.com/HEN3DRIK/BeadSorter/blob/main/beadsorter_schema.png?raw=true)

# Printing Guide
## Printing Linus Barth Parts
Print everything but the following Parts:
- 0-hopper-seq-plate.stl
- 0-hopper.stl
- 1-sequentializer-base.stl
- 1-sequentializer-ilc-corner.stl
- 1-sequentializer-tube-inset.stl
- 2-*.stl
- 3-analyzer-entry-tube.stl
- 4-dispatcher-tube-holder.stl
- 4-dispatcher-servo-holder.stl
- 4-dispatcher-base.stl
- 5-outputs-base.stl
- 5-outputs-slide-1.stl
- 5-outputs-slide-2.stl
- 5-outputs-slide-3.stl
- rpi-holder.stl

## Printing RichA777 parts
Print the following parts
- 0-hopper-seq-plate-4_holes_modified
- Sequencer_base_modified
- 0_Hopper_Modified_Cleaned_up

## Printing my parts
Print:
- 4x 1-sequentializer-ilc-corner_for_higher_motor.stl
- ring_container.stl
- 4_Stepper_shaft.stl
- Stepper_shaft_ring.stl
- Servo_base2.stl
- Stepper_shaft_ring.stl
- 3-analyzer-entry-tube_modified.stl
- UM2-Sequencer_tube_modified_tighter_Higher_Motor.stl
- 5-outputs-slide-1.stl
- 5-outputs-slide-2.stl
- 5-outputs-slide-3.stl
- holder_typeA.stl
- holder_typeB.stl
- Plate.stl
- Box_bottom.stl
- Box_lid.stl
- Box_top.stl
