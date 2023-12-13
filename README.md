# Self-Balancing-Triangle

ESP32 D2 controller, Simple FOC Mini BLDC driver, MT6701 magnetic encoder, EMAX 2806-100KV motor, MPU6050, 3 WS2812B leds, 3S 500 mAh LiPo battery.

Balancing controller can be tuned remotely over bluetooth (but do this only if you know what you are doing).

Example (change K1):

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

The same for K2, K3, K4. Send "d", "s", "a", .

<img src="/pictures/triangle1.jpg" alt="Self-Balancing-Triangle"/>
<img src="/pictures/triangle2.jpg" alt="Self-Balancing-Triangle"/>


If all leds are flashing red, the triangle needs to be calibrated. Connect via bluetooth to the controller. You will see a message that you need to calibrate the balancing points. 
Send c+ from serial monitor. This activate calibrating procedure. Set the triangle to first balancing point (green led on top). 
Hold still when the triangle does not fall to either side. Send c- from serial monitor. Set the triangle on second edge (green led on top). 
Send c- from serial monitor. Do the same with third edge. After calibrating, the triangle will begin to balance.

More about this:

https://youtu.be/dTYiqNcXw68

You will not find all functions in this code. Triangle can balance on any edge, but cannot stand up and can't roll over to the other edge.
In my video you can see that it is possible. But these functions are very difficult to tuning... Also many possible solutions how to do it. Maybe you can do this better?