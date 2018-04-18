# M2M-EMU-GPIO-Development
DigitalSensors (Emulated GPIO) development in M2M commboard

# TASKs TO-DO

Southbound functions to be developed

## Emulated GPIO configuration

`result emu_gpio_config(Signal, HW-Parse, Mode, Pull, Timming)`

*Inputs*

* Signal : Struct that will contains the Port & Pin
* HW-Parse : Mapping of the Hardware as PXY
* MODE: read / write
* Pull: yes / no
* Timming: Period

*Output*

* result: TODO Francis

## Set Mode
Change the mode of a PIN. It is called from config and form read_write

`result emu_gpio_set_mode(Signal, Mode)`

*Inputs*

* Signal : Struct that will contains the Port & Pin
* MODE: read / write


## Read Write digital secuence

`result emu_gpio_read_write(id_sensor, Stream_CS, Stream_SDI, Stream_SDO, Stream_SCK)`

*Inputs*

* Id_sensor: ID of the sensor. With this ID is possible to get all the characteristics of the pins (4) to be used
* Stream_XXX : Stream to be sent/read through each port.  It should have a minimum and a maximum longitude. Format: 
> * 1 / 0 : Write 1 or 0
> * X : Leave the pin as it is
> * R : Read the pin
> Examples of Stream: "1100", "XXX1", "1RRR". Each character takes a period of the signal. 

*Output*

* result: Stream read (Normally there is only one stream that read)




# Additional Information

These function(s) should be developed in the file **southbound_ec.c**

The main three functions of this development should be called from **main.c** with different options in order to demonstrate its behaviour and ease the testing.  

# Ressources

Structures to be used and examples:

TODO By Francis
