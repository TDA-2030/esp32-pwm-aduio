# _PWM Audio Example_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example will shows how to configure and operate the LEDC peripheral to play stereo audio  with 44.1KHz sample rate and 8-bit width.


## How to use example

### Hardware Required

* A development board with ESP32 SoC (e.g., ESP32-DevKitC, ESP-WROVER-KIT, etc.)
* A USB cable for power supply and programming
* Two speakers  or earphone

Connect two speakers to the following LEDC channels / individual GPIOs:

|ledc channel|GPIO|
|:---:|:---:|
|channel 0|GPIO25|
|channel 1|GPIO26|


```
             47R
GPIO17 +----/\/\/\----+
                      |    
                      | /|
                     +-+ |   Speaker
                     | | |     or
                     +-+ |  earphone
                      | \|
                      |
                      +--------------+ GND

             47R
GPIO18 +----/\/\/\----+
                      |    
                      | /|
                     +-+ |   Speaker
                     | | |     or
                     +-+ |  earphone
                      | \|
                      |
                      +--------------+ GND
```

### Configure the project

```
idf.py menuconfig
```

* Set serial port under Serial Flasher Options.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py flash monitor -b 921600
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

* You will hear a piece of music again and again from the speakers

you can also see the following output log on the serial monitor:

```
------------
main            11501931                <1%
IDLE1           1988064919              49%
IDLE0           1976562842              49%
Tmr Svc         31              <1%
ipc1            881             <1%
ipc0            687             <1%
esp_timer       1122            <1%
the memory get: 298248
```

## Troubleshooting

* Programming fail

    - Hardware connection is not correct: run `make monitor`, and reboot your board to see if there are any output logs.
    - The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.
* The voice is too small
    - Speaker impedance may be too large. Replace it with a higher impedance one
* The noise is too loud
    - A simple filter may be required
