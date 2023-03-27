# Si5351-GPS-10MHz

10 MHz Ref Clock with Si5351 drived by GPS<BR>
This repository was taken over by flafforque.
The Si5351-GPS-10MHz.ino file was revised because I didn't have an ST7789 display. My version is intended for a ST7735 TFT display. A rotary encoder or buttons are not connected, the routines for this are not required, but are present in the sketch. If a GPS signal is available, the current date and time are shown on the TFT display in the notation that is customary in Germany. There is an automatic summer/winter time changeover. Depending on the system, some jitter is to be expected with the Si5351A output frequency. However, the accuracy should be 0.1 ppm after a few minutes of operation.
