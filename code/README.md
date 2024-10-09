# Code Readme

## ESP32 Sensor Readings and Web Server

The ESP32 code responsible for sending data to our web server from the sensors on the ESP32 over UDP lies in [main.c](hurricane_box/main/main.c). The ESP32 has three tasks running during the program. The first task is for the timer which allows us to print out seconds for every reading. The second task is for the ADXL343 sensor. We use that sensor to calculate roll and pitch. Roll and pitch are used to detect vibrations. The third task is for adc devices which include the thermistor and the battery monitor. Thier respective pins are read into raw values and then coverted into usuable values. 

The node app responsible for the visualization of our data lives in [server.js](hurricane_box/node/server.js). It receives hurricane box data from the ESP32 over UDP and establishes a socket.io connection to the client-side file [index.html](hurricane_box/node/index.html) to send data over before rendering it on port 3000. 

On the client side, a corresponding socket.io connection is established to receive the remote hurricane box data and the resulting graphs are rendered through a CanvasJS script.

## PiCam Streaming

Within the pi_stream folder, there is another node app that renders a page on port 8080, which renders the HLS video stream produced by the PiCam on the Raspberry Pi.