# PRismino library

This document explains the usage of the library to be used with the [PRismino](https://github.com/Robopoly/PRismino) and [Robopoly Shield](https://github.com/Robopoly/Robopoly-Shield). These are additional functions to complement [Arduino functions](http://arduino.cc/en/Reference/HomePage).

**Warning**: some pins are hard-wired to elements that source current, this means that if the pins are set as outputs short-circuits can occur and **damage the micro-controller**, please read everything carefully to avoid it or you might break your PRismino.

## Installing

Installing the library is like with any [Arduino library](http://arduino.cc/en/Guide/Libraries), just copy `/libraries` and `/hardware` directories to your sketchbook directory:

* Windows: `My Documents\Arduino\`
* Mac: `~/Documents/Arduino/`
* Linux: `~/Documents/Arduino/`

## Motor functions

### Wheels/H-bridge

`void setSpeed(int8_t leftSpeed, int8_t rightSpeed);`

    // set the left wheel forward 40% and right wheel backwards 40%
    setSpeed(40, -40);

Defines the motor speed in percent from -100 to 100 trough the H-bridge. It uses the pins 9, 10, 11 and 12 of the PRismino. The speed is defined by a PWM signal with a period of 15'625Hz. This function uses the 10-bit `Timer/Counter 4` of the ATmega32U4. In order to change direction interrupt vectors select which pin must be toggled.

In order for the command to take effect a short delay is required between 2 `setSpeed()` commands, 100ms (`delay(100)`) works quite well. This is typically needed for closed-loop regulation system where fast `setSpeed()` calls won't have the desired effect when called too often.

The H-bridge component is the [DRV8833](http://www.ti.com/product/drv8833) which allows slow and fast decay modes, the library uses the fast decay mode by default. To select the slow decay mode one can define `SLOWDECAY` before including `prismino.h` file, or uncomment it in the `prismino.h` file.

    // use the slow decay mode instead of fast decay
    #define SLOWDECAY
    #include <prismino.h>

The H-bridge can deliver **2A per channel**, it is temperature regulated so if it overheats it will automatically stop and wait until the IC temperature gets low enough to restart.

## Button/switch functions

### DIP-switch

`void dipSwitch(uint8_t dipId, func_t callbackFunction = NULL, uint8_t interruptMode = CHANGE);`
    
    // call a function when switch 1 is toggled
    void myFunction(void)
    {
        // ...
    }
    dipSwitch(DIP1, myFunction, CHANGE);
    
    // call a function when switch 2 logic level goes from 1 to 0 (falling edge)
    void myFunction2(void)
    {
        // ...
    }
    dipSwitch(DIP2, myFunction2, FALLING);
    
    // to unregister just call the same function without the callback function
    dipSwitch(DIP2, NULL);
    // or simply
    dipSwitch(DIP2);

Registers the callback function when a DIP-switch is toggled. It is best to call it in the initial setup part of the code (inside the `setup()` function). Different modes are:

* `LOW`: a low logic level triggers the interrupt, this keeps firing the interrupt vector if the pin value is at logic 0
* `CHANGE`: either transition triggers the interrupt
* `FALLING`: a high to low logic level transition triggers the interrupt
* `RISING`: a low to high logic level transition triggers the interrupt

The callback function cannot have any arguments.

The switches are located on pins 0 to 3, so when they are soldered the **pins cannot be used in output mode**.

The `DIP*` definitions help to find the right switch, they should be used instead of the pin numbers.

| Switch | Definition  | Value (pin) | Port | Vector      |
| ------ | ----------- | ----------- | ---- | ----------- |
| 1      | `DIP1`      | 0           | PD2  | `INT2_vect` |
| 2      | `DIP2`      | 1           | PD3  | `INT3_vect` |
| 3      | `DIP3`      | 2           | PD1  | `INT1_vect` |
| 4      | `DIP4`      | 3           | PD0  | `INT0_vect` |

### Button

`void buttonCallback(func_t callbackFunction = NULL);`

    // call a function when the shield button is pressed
    void buttonClick(void)
    {
        // ...
    }
    buttonCallback(buttonClick);
    
    // to unregister just call the same function without the callback function
    buttonCallback(NULL);

Registers a callback function for when the shield button is clicked, it is connected to the pin 7 on the shield. It defaults to a falling edge interrupt as the pin is pulled high by the micro-controller internal pull-up resistor when the button is released.

**Note**: mind the [contact bounce](http://en.wikipedia.org/wiki/Switch#Contact_bounce) which can be annoying, but [preventable](http://arduino.cc/en/Tutorial/Debounce).

## Timed functions

Sometimes it's useful to call a function regularly, such as to check a sensor every 10 seconds or follow a line while checking for obstacles every 500ms. A timed function lets the user set up an automatic call to a function at predetermined intervals.

The time interval is a multiple of 100ms.

`int8_t setTimer(func_t callbackFunction, uint16_t interval, uint8_t callNumber = 0);`
    
    // call a function every 5*100ms = 500ms, 10 times
    void myFunction(void)
    {
        // ... this code must be executed in under 100ms
    }
    setTimer(myFunction, 5, 10);
    
    // call a function every 600*100ms = 60 seconds = 1 minute, indefinitely
    void myFunction2(void)
    {
        // ...
    }
    setTimer(myFunction2, 600);

This function uses the `Timer/Counter 1`, the same as the servo function, so the **servo motors cannot be used while this is being used**.

As the delay between the calls (`interval`) is coded on a 16-bit unsigned integer the maximum time between calls can be at most 2^16ms = 65536ms = 65.536s, so a little over a minute.

The function can be called at most 256 times if the `callNumber` parameter is provided, otherwise it will be called indefinitely.

To unset a timed function use `void unsetTimer(uint8_t);`, `setTimer()` returns an identification number that can be used to unset it later.

    // set and then unset a timed function
    void myFunction3(void)
    {
        // ...
    }
    uint8_t myTimedFunctionID = setTimer(myFunction3, 10);
    
    unsetTimer(myTimedFunctionID);

If the timed function slots are full `setTimer()` will return a `-1`, the maximum number of timed functions can be set in the `prismino.h` file with the `TIMEDFUNCTIONS` definition which is by default set to 4.

**NOTE**: the way the program calls the functions requires all of the timed functions combined to take less than 100ms in execution time, so avoid `delay()` functions inside of your routine calls and only use the `setTimer()` for small code bits.

## Miscellaneous functions

### Sound

`void play(uint16_t frequency, uint16_t time);`

    // play a 440Hz sound during 1 second
    play(440, 1000);

Sends a square wave to the [Robopoly shield's](https://github.com/Robopoly/Robopoly-Shield) buzzer (pin 8) to output a sound. The parameters are the frequency in Hz and the play-time in milliseconds.

It's actually a macro to the Arduino [`tone()`](http://arduino.cc/en/Reference/tone) function, it defines the buzzer pin so only 2 arguments are needed.

## Definitions

These shortcuts, that are included in the library, can be used to make the code more clean:

### Servo pins

The numbers correspond to the markings on the PCB silkscreen.

    #define SERVO1 6 // PORTD7
    #define SERVO2 5 // PORTC6

### DIP switch pins

Do not use these pins as outputs! The reason for this convoluted order is because they were easier to route on the PCB this way.

    #define DIP1 2 // PORTD0
    #define DIP2 3 // PORTD1
    #define DIP3 1 // PORTD3
    #define DIP4 0 // PORTD2

### Button pin

Do not use this pin as an output!

    #define BUTTON 7 // PORTE6

### Buzzer pin

    #define BUZZER 8 // PORTB4

### LED pin

    #define LED 13 // PORTC7

### Potentiometer pin

Do not use the potentiometer pin as output when the shield is connected, it can damage the MCU!

    #define POT A0 // PORTF7

See the button/switch functions for more details about the interrupt type definitions.

# Examples

A few example programs are also provided to show how to use these functions and all the components on the Robopoly Shield. See them under _File -> Examples -> PRismino_ in Arduino IDE.

# Licence

This software is published under [LGPL](http://www.gnu.org/licenses/lgpl.html).