# GPS-Speedometer
I wanted a gps speedometer for my old motorcycle, this is it!

I built this using a 128x64 pixel OLED display in portrait mode, so rather 64x128 in my case
Also there is a regular GPS module (RX/TX comm) could of course be modified for a i2c module also.

My build:
- Wemos D1 mini (clone)
- RTC working over i2c
- 128x64 OLED 2.42" SSD130x over i2c
- Thermometer using LM75A over i2c
- GPS module using rx/tx comm.
- All connected using 5v from the wemos over usb on my motorcycle
