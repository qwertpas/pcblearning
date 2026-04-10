# pcblearning
A collection of PCB and firmware projects for learning PCB design. Projects are designed to have few components, cheap, yet rich in potential features.

## Using common libraries
1. After creating a new project in KiCad, copy the contents of `copy-into-your-project` into the same folder that contains your project's `.kicad_pcb` file.
2. Restart KiCad. When you place new components in the schematic or PCB editor, the `pcblearning` library should be available.

## minrc1
Tiny wheeled robot with two PM08 stepper motors, three microphones, and a buzzer. ESP32S3 supermini dev board allows for wireless control, USB-C battery charging, and running small neural networks. Microphones and buzzer may be useful for locating other robots, echolocation, and detecting surface properties (through vibration/noise from ground contact).
![minrc1](minrc1/media/minirc_4pic.jpg)
![schematic](minrc1/media/minirc1_schematic.jpg)

## minrc-ref
Even more minimal than minrc1: just an ESP32S3-supermini, 4x DRV8837 motor drivers, 2x stepper motors, and resistor divider voltage sense. Intended as a reference design to be used as a starting point.