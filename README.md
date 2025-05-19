                            ------Steps for installing the library------
    
    1. Download the zip file containing all the files
    2. Extract it
    3. Copy the directory in the libraries folder where all the Arduino libraries belong 
        Usually:
            Windows - Documents/Arduino/libraries/
            Mac - ~/Arduino/libraries/
    4. Restart Arduino IDE
    5. Use #include <LD06.h> in your Sketch
                                        YOU ARE READY TO GO!
    
    
                                    -------Using Tips-------

    First you have to assure that the LD06 sensor is connected correctly;
        P5V  - 5V voltage
        GND  - Ground
        DATA - RX (Any serial data recieving Pin)
        CTL  - PWM (You cand leave it unconnected because the default motor speed is well balanced already)
    
    Create a Searial Port for communication (ex: HardwareSerial mySerial(2); )
    Create a lidar object (ex: LD06 lidar(mySerial, 3); 3 is the chosen RX pin)
    
    In void setup() you have to set up the lidar:
        lidar.setup(); 

    In void loop() you can use the following methods:
        lidar.getMap(RANGE, GRID_SIZE);
        lidar.getMap(RANGE, GRID_SIZE, START_ANGLE, END_ANGLE);
        lidar.getIndedPoints(RANGE, GRID_SIZE);
        lidar.getRawPoints(RANGE);



                            -------Sensor Hardware Specifications------

    Model:                  LD06
    Sensor Type:            2D Rotating LiDAR (360°)
    Measurable Distance:    0.18 m – 12 m
    Accuracy:               ±2 cm (for distances < 2 m) 
    Angular Resolution:     ~0.5°
    Supply Voltage:         5V 
    Operating Current:      ~300 mA 
    Interface:              UART (3.3V logic level) 
    Baud Rate:              230400 bps 
    Rotation Speed:         ~450 RPM 
    Dimensions:             48 mm x 48 mm x 68 mm
    Weight:                 120 g



