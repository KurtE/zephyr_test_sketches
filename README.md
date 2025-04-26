# Zephyr_test_sketches

This github project contains some experiments that a few of us are trying out as a method of learning 
about using the Zephyr system directly.

Up till now My only testing and learning of zeyphyr was through helping test and in some cases
add fixes or enhancements to the Arduino wrapper of the ArduinoCore-Zephyr project.

So far I have mainly concentrated on how to use the Teensy 4.x board (boards\pjrc) types within Zephyr and
while doing so adding support for the Teensy Micromod.

** WARNING ** - These are only experiments that a few of us are playing with.  So absolutely no promises
or guarantees that any of this is useful!


https://github.com/Zephyrproject-rtos/Zephyr/pull/88502

I believe this one is close to being pulled in, just going through all the conformance testing and reviews.

Currently I have three test sketches, which are not much, but with each one I learned more about Zephyr.  None of them
are that useful other than examples of some different pieces. 

At some point I will probably clean this up.  With each test app, I simply copy the previous one, make changes and once
I am satisfied that I have some feature somewhat working I go on to the next one.

At some point will extract all the duplicate files, into some common directory...  But then again...

First one was:

## USBToSerial

I started off with the Zephyr sample usb\cdc_acm - and had it echo what you typed in, back to itself.

I then added UART support.  I created a quick and dirty UARTDevice class which is modeled
after the Arduino Serial class, but again just quick and dirty, I did not add in classes like Print or Stream,
but added a few basic print functions... Actually, a quick and real dirty printf, which uses sprintf to a buffer
and then writes the buffer out the the uart... 

The overlay has which Uart I wish to use the above class to wrap it as well as wrap the USB one.  The one
I have chosen is logically the Serial2 on Teensy 4.x boards (pins 7 and 8), and the code 
simply reads from one and writes to the other...


## Zephyr-sd

This is just a simple sketch that tries to read an SD card in an SDIO adapter, which is natively on Teensy 4.1 boards
and on my Teensy Micromod adapters and some earlier adapter boards I made for the T4.0 (bottom pins)

## ILI9341_plus

Test sketch to see if I could talk to an SPI port and drive an ILI9341 display using SPI code, where the driver
is not part of Zephyr.  That is instead of using the ADAFruit_2_8_tft_touch_v2 shield, which I think then uses LVGL
I wanted to use my own code. 

We have partially implemented a port of my ili9341 library that I had earlier ported to GIGA and then GIGA on zepyr
to Zephyr.  There were lots of things to learn here about using SPI, and we now have a partially working version of
the graphics test.  

# What's next? 
Good question!

