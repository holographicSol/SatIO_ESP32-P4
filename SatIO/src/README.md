                                  SatIO - Written by Benjamin Jack Cullen.

              A general purpose programmable I/O platform for automation and manual throughput.
    Supporting stacks (up to 10 functions per output pin) of logic across 20 output pins on the portcontroller.

  What can SatIO tell you is true? Potentially infinite things.

  Applications? Potentially infinite applications.

  Limitations? Being vastly general purpose comes at a perfromnce cost. SatIO is designed to be vastly general purpose.

  Inference in Bayesian Reasoning? Moon tracking for example can be used to track the moon, it can also be used for one example; to 
  track the tides, if the system is aware moon/planetary positioning and datetime then marine life values may also be inferred relative
  to the inferred tide values and known datetime. There is a lot of data that can be used in many ways, with a kind of network effect.
  Or more simply 'SatIO is one hell of a switch'.

  The amount of data and I/O available on this project makes this project a more than adequately general purpose platform for all
  kinds of future projects, from universally aware LLMs harnessing SatIO's optional serial output, to sensor drones, or even just a
  GPS syncronized astro clock. Everything other than the ESP32 running SatIO should be considered as optional and modular, meaning
  SatIO builds can be from a headless chip with GPS pouring out data over serial, or a full system with extra I/O and display.

  Short of quantum navigation on a microchip, GPS is currently used for navigation, providing values that many more values
  can be calculated from, providing there is not something potentially terminally wrong with the universe.

  Matrix logic is an attempt to maximize programmable potential and hardware configuration is designed to attempt maximum IO potential.
  If more output is needed then add another I2C port controller, if more input is needed then add another custom I2C sensor module.
  If more matrix logic is needed then build on another MCU, matrix switches and switch functions have been balanced to allow for plenty
  of switches and functions for said switches, with regards to available memory/storage (10 functions per switch not including switch linking
  where logic can be stacked accross multiple/all switches/memory allocations).

        Design: Break out all the things and build I2C peripherals as required to orbit the ESP32/Central-MCU.

                                Wiring For Keystudio ESP32 PLUS Development Board

                                ESP32: 1st ATMEGA2560 with shield as Port Controller (not on multiplexer):
                                ESP32: I2C SDA -> ATMEGA2560: I2C SDA
                                ESP32: I2C SCL -> ATMEGA2560: I2C SCL

                                ESP32: 2nd ATMEGA2560 with shield as Control Panel (not on multiplexer):
                                ESP32: io25    -> ATMEGA2560: io22
                                ESP32: I2C SDA -> ATMEGA2560: I2C SDA
                                ESP32: I2C SCL -> ATMEGA2560: I2C SCL

                                Other ESP32 i2C Devices (not on multiplexer):
                                ESP32: SDA0 SCL0 -> DS3231 (RTC): SDA, SCL (5v)

                                ESP32: WTGPS300P (5v) (for getting a downlink):
                                ESP32: io27 RXD -> WTGPS300P: TXD
                                ESP32: null TXD -> WTGPS300P: RXD

                                ESP32 i2C: i2C Multiplexing (3.3v) (for peripherals):
                                ESP32: i2C -> TCA9548A: SDA, SCL

                                ESP32: Analog/Digital Multiplexing (3.3v) (for peripherals):
                                ESP32: io4    -> CD74HC4067: SIG
                                ESP32: io32   -> CD74HC4067: S0
                                ESP32: io33   -> CD74HC4067: S1
                                ESP32: io16   -> CD74HC4067: S2
                                ESP32: io17   -> CD74HC4067: S3
                                CD74HC4067 C0 -> DHT11: SIG

                                ESP32 VSPI: SDCARD (5v) (for matrix and system data):
                                ESP32: io5  -> HW-125: CS (SS)
                                ESP32: io23 -> HW-125: DI (MOSI)
                                ESP32: io19 -> HW-125: DO (MISO)
                                ESP32: io18 -> HW-125: SCK (SCLK)

                                ESP32 HSPI: SSD1351 OLED (5v) (short wires recommended):
                                ESP32: io14 -> SSD1351: SCL/SCLK
                                ESP32: io12 -> SSD1351: MISO/DC
                                ESP32: io13 -> SSD1351: SDA
                                ESP32: io26 -> SSD1351: CS

                                           $SATIO SENTENCE
                              
                              RTC Sync Time (UTC)             System Uptime (Seconds)
                              |      RTC Sync Date (UTC)      |
        Tag                   |      |                        |   Longitude Degrees
        |                     |      |                        |   |
        $SATIO,000000,00000000,000000,00000000,000000,00000000,0,0,0,*CHECKSUM
              |      |                        |      |          |
              |      |                        |      |          Latitude Degrees
              |      RTC Date (UTC)           |      Local Date (UTC Offset)
              RTC Time (UTC)                  Local Time (UTC Offset)

                                          $MATRIX SENTENCE 

                                                                              Matrix Switch Output Port 19
                                                                              |
                                                                              |    Matrix Switch State 0
                                                                              |    |
    $MATRIX,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,*CHECKSUM
           |                                                                                                                                                   |
          Matrix Switch Output Port 0                                                                                                                          Matrix Switch State 19
                                                                                          

                                          $SENSORS SENTENCE

                      Sensor 0
                      |
              $SENSORS,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,*CHECKSUM
                                                                                  |
                                                                                  Sensor 15


                                          $SUN SENTENCE
                                                    
                                    Right Ascension 
                                    |       Azimuth 
                                    |       |       Rise
                                    |       |       |
                                $SUN,0.0,0.0,0.0,0.0,0.0,0.0,*CHECKSUM
                                        |       |       |
                                        |       |       Set 
                                        |       Altitude
                                        Declination 


                                          $MOON SENTENCE

                                                Rise
                                Right Ascension |
                                |       Azimuth | 
                                |       |       |       Phase
                                |       |       |       |
                          $MOON,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,*CHECKSUM
                                    |       |       |       |
                                    |       |       Set     Luminessence
                                    |       Altitude
                                    Declination


                                        $MERCURY SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
              $MERCURY,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude  


                                         $VENUS SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
                $VENUS,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude  


                                        $MARS SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
                 $MARS,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude


                                      $JUPITER SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
              $JUPITER,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude


                                      $SATURN SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
               $SATURN,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude


                                      $URANUS SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
               $URANUS,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude 


                                      $NEPTUNE SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
              $NEPTUNE,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude  


  
    Summary: Over one quintillion possible combinations of stackable logic across 20 switches for a general purpose
    part or standalone device.
  
    Whats the point? Working with ESP32 is cheap and from this project I intend to have reusable, general purpose parts
    as modules that can work both together and standalone, creating a platform I can go to when working with ESP32.
  
    Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
    of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html).
    Additions: 1: doXRiseSetTimes(). This allows for calculating rise and set times of all planets and objects according to time and location.
               2: inRange60(). Ensures minutes and second values are wihin 0-59 for planet/object rise, set times.
               3: inRange24(). Ensures hour values are wihin 0-23 for planet/object rise, set times.
  
    ToDo: 20 programmable modulators on each output pin on the port controller. 
    
    ToDo: Terrain elevation: Experiments have been made decompressing NASA's SRTMGL1 (Shuttle Radar Topography Mission) files quickly.
  
    ToDo: More data and calculate more data from existing data.
  
    ToDo: Macros.
  
    Complete PlatformIO project files, libraries and modified libraries:
    https://drive.google.com/drive/folders/13yynSxkKL-zxb7iLSkg0v0VXkSLgmtW-?usp=sharing
  
    Builds:
    Nano SatIO (Passive): Serial input/output only, headless. Useful for LLMs and things.
    Full SatIO (Active): Everything.

-----

[ MATRIX SWITCH LOGIC ]

Logic may require or not require values X,Y,Z.
    
[Available Matrix Switch Functions]

    [0] None
    [1] On
    [2] SwitchLink
    [3] LocalTime
    [4] Weekday
    [5] DateDayX
    [6] DateMonthX
    [7] DateYearX
    [8] DegLat
    [9] DegLon
    [10] DegLatLon
    [11] INSLat
    [12] INSLon
    [13] INSLatLon
    [14] INSHeading
    [15] INSSpeed
    [16] INSAltitude
    [17] UTCTimeGNGGA
    [18] PosStatusGNGGA
    [19] SatCount
    [20] HemiGNGGANorth
    [21] HemiGNGGASouth
    [22] HemiGNGGAEast
    [23] HemiGNGGAWest
    [24] GPSPrecision
    [25] AltGNGGA
    [26] UTCTimeGNRMC
    [27] PosStatusGNRMCA
    [28] PosStatusGNRMCV
    [29] ModeGNRMCA
    [30] ModeGNRMCD
    [31] ModeGNRMCE
    [32] ModeGNRMCN
    [33] HemiGNRMCNorth
    [34] HemiGNRMCSouth
    [35] HemiGNRMCEast
    [36] HemiGNRMCWest
    [37] GSpeedGNRMC
    [38] HeadingGNRMC
    [39] UTCDateGNRMC
    [40] LFlagGPATT
    [41] SFlagGPATT
    [42] RSFlagGPATT
    [43] INSGPATT
    [44] SpeedNumGPATT
    [45] MileageGPATT
    [46] GSTDataGPATT
    [47] YawGPATT
    [48] RollGPATT
    [49] PitchGPATT
    [50] GNGGAValidCS
    [51] GNRMCValidCS
    [52] GPATTValidCS
    [53] GNGGAValidCD
    [54] GNRMCValidCD
    [55] GPATTValidCD
    [56] Gyro0AccX
    [57] Gyro0AccY
    [58] Gyro0AccZ
    [59] Gyro0AngX
    [60] Gyro0AngY
    [61] Gyro0AngZ
    [62] Gyro0MagX
    [63] Gyro0MagY
    [64] Gyro0MagZ
    [65] Gyro0GyroX
    [66] Gyro0GyroY
    [67] Gyro0GyroZ
    [68] Meteors
    [69] SunAz
    [70] SunAlt
    [71] MoonAz
    [72] MoonAlt
    [73] MoonPhase
    [74] MercuryAz
    [75] MercuryAlt
    [76] VenusAz
    [77] VenusAlt
    [78] MarsAz
    [79] MarsAlt
    [80] JupiterAz
    [81] JupiterAlt
    [82] SaturnAz
    [83] SaturnAlt
    [84] UranusAz
    [85] UranusAlt
    [86] NeptuneAz
    [87] NeptuneAlt
    [88] ADMPlex0
    [89] MappedValue
    [90] SDCARDInserted
    [91] SDCARDMounted

[Available Switch Function Operators]
    [0] None
    [1] Equal
    [2] Over
    [3] Under
    [4] Range

-----

[SERIAL]

System

    system --save
    system --load
    system --restore-defaults

Mapping

    mapping --save
    mapping --load
    mapping --delete
    mapping -s n       Specify map slot n.
    mapping -m n       Specify slot -s mode. (0 : map min to max) (1 : center map x0) (2 : center map x1)
    mapping -c0 n      Configuration map slot -s  value to map. See available map values.
    mapping -c1 n      Configuration map slot -s. (mode 0 : in_min)  (mode 1 : approximate center value)
    mapping -c2 n      Configuration map slot -s. (mode 0 : in_max)  (mode 1 : Neg_range : 0 to approximate center value)
    mapping -c3 n      Configuration map slot -s. (mode 0 : out_min) (mode 1 : Pos_range : ADC max - neg range)
    mapping -c4 n      Configuration map slot -s. (mode 0 : out_max) (mode 1 : out_max)
    mapping -c5 n      Configuration map slot -s. (mode 1 only : DEADZONE : expected flutuation at center)

example map analog stick axis x0 on admplex0 channel 0 into map slot 0:
    mapping -s 0 -m 1 -c0 16 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
example map analog stick axis x1 on admplex0 channel 1 into map slot 1:
    mapping -s 1 -m 2 -c0 17 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50

Matrix

    matrix --new                Clears matrix in memory.
    matrix --save n             Specify file slot.
    matrix --load n             Specify file slot.
    matrix --delete n           Specify file slot.
    matrix --startup-enable
    matrix --startup-disable
    matrix -s n                 Specify switch index n.
    matrix -f n                 Specify function index n.
    matrix -p n                 Set port for switch -s.
    matrix -fn n                Set function -f for switch -s. See available matrix functions.
    matrix -fx n                Set function -f value x for switch -s.
    matrix -fy n                Set function -f value y for switch -s.
    matrix -fz n                Set function -f value z for switch -s.
    matrix -fi n                Set function -f logic inverted for switch -s.
    matrix -fo n                Set function -f operator for switch -s.
    matrix --pwm0 n             Set switch -s uS time off period (0uS = remain on)
    matrix --pwm1 n             Set switch -s uS time on period  (0uS = remain off after on)
    matrix --flux n             Set switch -s output fluctuation threshold.
    matrix --oride n            Override switch -s output values.
    matrix --computer-assist n  Enable/disable computer assist for switch -s.
    matrix --omode n            Set switch -s output mode: (0 : matrix logic) (1 : mapped value analog/digital).
    matrix --map-slot n         Set switch -s output as map slot n value.

example default all switch configurations:

    matrix --new
example stat switch zero:

    stat --matrix 0
example set switch zero:

    matrix -s 0 -f 0 -p 33 -fn 91 -fx 1 -fo 1 --pwm0 1000000 --pwm1 15000 --computer-assist 1
example set mapped output mode:
    matrix -s 0 --omode 1

example set matrix logic output mode:
    matrix -s 0 --omode 0

INS

    ins -m n              Set INS mode n. (0 : Off) (1 : Dynamic, set by gps every 100ms) (2 : Fixed, remains on after conditions met).
    ins -gyro n           INS uses gyro for attitude. (0 : gyro heading) (1 : gps heading).
    ins -p n              Set INS mimimum required gps precision factor to initialize.
    ins -s n              Set INS mimimum required speed to initialize.
    ins -r n              Set INS maximum required heading range difference to initialize (difference between gps heading and gyro heading).
    ins --reset-forced n  Reset INS remains on after conditions met.

Satio

    satio --speed-units n  Set displayed units (0 : M/S) (1 : MPH) (2 : KPH) (3 : KTS estimated)
    satio --utc-offset n   Set +-seconds offset time.
    satio --mode-gngga     Use GNGGA data for location.
    satio --mode-gnrmc     Use GNRMC data for location.

Gyro

    gyro --calacc        Callibrate the accelerometer.
    gyro --calmag-start  Begin calibrating the magnetometer.
    gyro --calmag-end    End calibrating the magnetometer.

SDCard

    sdcard --mount
    sdcard --unmount

Other

    -v    Enable verbosoity.
    -vv   Enable extra verbosoity.
    help

Stat

    stat -e     Enable print.
    stat -d     Disable print.
    stat -t     Enables/disables serial print stats and counters. Takes arguments -e, -d.
    stat --partition-table      Print partition table.
    stat --memory-ram           Print ram information.
    stat --sdcard               Print matrix information.
    stat --system               Print system configuration.
    stat --matrix               Print matrix configuration.
    stat --matrix n             Print matrix switch n configuration.
    stat --matrix -A            Print configuration of all matrix switches.
    stat --mapping              Print configuration of all mapping slots.
    stat --sentence -A          Print all sentences. Takes arguments -e, -d.
    stat --sentence --satio     Takes arguments -e, -d.
    stat --sentence --ins       Takes arguments -e, -d.
    stat --sentence --gngga     Takes arguments -e, -d.
    stat --sentence --gnrmc     Takes arguments -e, -d.
    stat --sentence --gpatt     Takes arguments -e, -d.
    stat --sentence --matrix    Takes arguments -e, -d.
    stat --sentence --admplex0  Takes arguments -e, -d.
    stat --sentence --gyro0     Takes arguments -e, -d.
    stat --sentence --sun       Takes arguments -e, -d.
    stat --sentence --moon      Takes arguments -e, -d.
    stat --sentence --mercury   Takes arguments -e, -d.
    stat --sentence --venus     Takes arguments -e, -d.
    stat --sentence --mars      Takes arguments -e, -d.
    stat --sentence --jupiter   Takes arguments -e, -d.
    stat --sentence --saturn    Takes arguments -e, -d.
    stat --sentence --uranus    Takes arguments -e, -d.
    stat --sentence --neptune   Takes arguments -e, -d.
    stat --sentence --meteors   Takes arguments -e, -d.

-----