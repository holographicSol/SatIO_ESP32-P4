![plot](./Data/UnidentifiedStudios.png)

  SatIO - Written by Benjamin Jack Cullen.

-----

  A general purpose programmable I/O platform for automation, throughput and LLM's.

  A huge matrix switch in a small package, supporting stacks of logic across
  70 output pins and 100 mapping slots.

  For ESP32-P4

-----

  Inference in Bayesian Reasoning? Moon tracking for example can be used to track the moon, it can also be used for one example; to 
  track the tides, if the system is aware of moon/planetary positioning and datetime then marine life values may also be inferred
  relative to the inferred tide values and known datetime. There is a lot of data that can be used in many ways, with a kind of network
  effect. Or more simply 'SatIO is one hell of a switch'.

-----

[ MATRIX SWITCH LOGIC ]

Logic may require or not require values X,Y,Z.

All of the following values can be used in the matrix:

        [0] NONE
        [1] ON
        [2] SWITCHLINK
        [3] LOCALTIME
        [4] WEEKDAY
        [5] DATEDAYX
        [6] DATEMONTHX
        [7] DATEYEARX
        [8] DEGLAT
        [9] DEGLON
        [10] INSLAT
        [11] INSLON
        [12] INSHEADING
        [13] INSALTITUDE
        [14] POSSTATUSGNGGA
        [15] SATCOUNT
        [16] GPSPRECISION
        [17] ALTGNGGA
        [18] GSPEEDGNRMC
        [19] HEADINGGNRMC
        [20] LFLAGGPATT
        [21] SFLAGGPATT
        [22] RSFLAGGPATT
        [23] INSGPATT
        [24] SPEEDNUMGPATT
        [25] MILEAGEGPATT
        [26] GSTDATAGPATT
        [27] YAWGPATT
        [28] ROLLGPATT
        [29] PITCHGPATT
        [30] GNGGAVALIDCS
        [31] GNRMCVALIDCS
        [32] GPATTVALIDCS
        [33] GNGGAVALIDCD
        [34] GNRMCVALIDCD
        [35] GPATTVALIDCD
        [36] GYRO0ACCX
        [37] GYRO0ACCY
        [38] GYRO0ACCZ
        [39] GYRO0ANGX
        [40] GYRO0ANGY
        [41] GYRO0ANGZ
        [42] GYRO0MAGX
        [43] GYRO0MAGY
        [44] GYRO0MAGZ
        [45] GYRO0GYROX
        [46] GYRO0GYROY
        [47] GYRO0GYROZ
        [48] METEORS
        [49] SUNAZ
        [50] SUNALT
        [51] MOONAZ
        [52] MOONALT
        [53] MOONPHASE
        [54] MERCURYAZ
        [55] MERCURYALT
        [56] VENUSAZ
        [57] VENUSALT
        [58] MARSAZ
        [59] MARSALT
        [60] JUPITERAZ
        [61] JUPITERALT
        [62] SATURNAZ
        [63] SATURNALT
        [64] URANUSAZ
        [65] URANUSALT
        [66] NEPTUNEAZ
        [67] NEPTUNEALT
        [68] HEMIGNGGANORTH
        [69] HEMIGNGGASOUTH
        [70] HEMIGNGGAEAST
        [71] HEMIGNGGAWEST
        [72] POSSTATUSGNRMCA
        [73] POSSTATUSGNRMCV
        [74] MODEGNRMCA
        [75] MODEGNRMCD
        [76] MODEGNRMCE
        [77] MODEGNRMCN
        [78] HEMIGNRMCNORTH
        [79] HEMIGNRMCSOUTH
        [80] HEMIGNRMCEAST
        [81] HEMIGNRMCWEST
        [82] ADMPLEX0
        [83] MAPPEDVALUE
        [84] SDCARDINSERTED
        [85] SDCARDMOUNTED
        [86] PCINPUTVALUE

-----

[ Available Switch Function Operators ]

Many matrix functions accept operators, where required:

    [0] None
    [1] Equal
    [2] Over
    [3] Under
    [4] Range

-----

System

    system --save               Takes no further arguments.
    system --load               Takes no further arguments.
    system --restore-defaults   Takes no further arguments.
    system -log                 Automatically log data to disk (See performance for timing). Takes arguments -e, -d.

-----

Mapping

Many values can be mapped and then used in the matrix and or sent directly to the port controller.

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

-----

Matrix

Setup the matrix:

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

    matrix -s 0 -f 0 -p 33 -fn 85 -fx 1 -fo 1 --pwm0 1000000 --pwm1 15000 --computer-assist 1

example set mapped output mode:

    matrix -s 0 --omode 1

example set matrix logic output mode:

    matrix -s 0 --omode 0

-----

INS

Customizable Inertial navigation system.

    ins -m n              Set INS mode n. (0 : Off) (1 : Dynamic, set by gps every 100ms) (2 : Fixed, remains on after conditions met).
    ins -gyro n           INS uses gyro for attitude. (0 : gyro heading) (1 : gps heading).
    ins -p n              Set INS mimimum required gps precision factor to initialize.
    ins -s n              Set INS mimimum required speed to initialize.
    ins -r n              Set INS maximum required heading range difference to initialize (difference between gps heading and gyro heading).
    ins --reset-forced n  Reset INS remains on after conditions met.

-----

Satio

      satio --coord-mode-gps           Use GPS latitude, longitude values.
      satio --coord-mode-static        Do not update latiude, longitude unless --set-coord or otherwise.
      satio --set-coord -lat n -lon n  Set degrees latitude and longitude (ensure --coord-mode-static before --set-coord).
      satio --utc-offset n             Set +-seconds offset time.
      satio --auto-datetime-on         Enable set datetime automatically  (--auto-datetime-on overrides any datetime -set).
      satio --auto-datetime-off        Disable set datetime automatically (ensure --auto-datetime-off before using -set time).
      satio --set-datetime --year n --month n --mday n --hour n --minute n --second n  (must be UTC except if utc offset 0).

      satio --speed-mode-gps     Use GPS speed values.
      satio --speed-mode-static  Do not update speed unless --set-speed or otherwise.
      satio --set-speed n        Set speed in meters per second (ensure --speed-mode-static before --set-speed).
      satio --speed-unit-KTS     Use default knots.
      satio --speed-unit-KPH     Convert knots per second to K/PH.
      satio --speed-unit-MPH     Convert knots per second to M/PH.
      satio --speed-unit-mPS     Convert knots per second to meters per second.

      satio --altitude-mode-gps         Use GPS altitude values.
      satio --altitude-mode-static      Do not update speed unless --set-altitude or otherwise.
      satio --set-altitude n            Set altitude in meters (ensure --altitude-mode-static before --set-altitude).
      satio --altitude-unit-meters      Use default meters altitude.
      satio --altitude-unit-kilometers  Convert meters to kilometers.
      satio --altitude-unit-miles       Convert meters to miles.

      satio --ground-heading-mode-gps     Use GPS ground heading values.
      satio --ground-heading-mode-static  Do not update heading unless --set-ground-heading or otherwise.
      satio --set-ground-heading          Set ground heading in degrees (0-360. Ensure --ground-heading-mode-static before --ground-heading).

-----

Gyro

    gyro --calacc        Callibrate the accelerometer.
    gyro --calmag-start  Begin calibrating the magnetometer.
    gyro --calmag-end    End calibrating the magnetometer.

-----

SDCard

    sdcard --mount
    sdcard --unmount

-----

Performance

    powercfg --ultimate-performance  Sets power configuration to ultimate performance mode.
    powercfg --power-saving          Sets power configuration to low power consumption mode.
    settick -e                       Enable tick delay for specified args.
    settick -d                       Disable tick delay for specified args (enables millisecond delay).
    settick --admplex0               Takes arguments -e, -d.
    settick --gyro0                  Takes arguments -e, -d.
    settick --universe               Takes arguments -e, -d.
    settick --gps                    Takes arguments -e, -d.
    settick --matrix                 Takes arguments -e, -d.
    settick --pcinput                Takes arguments -e, -d.
    settick --log                    Takes arguments -e, -d.

    example: settick -e --admplex0 --gyro0 --gps

    setdelay --admplex0               Specify ticks/milliseconds delay.
    setdelay --gyro0                  Specify ticks/milliseconds delay.
    setdelay --universe               Specify ticks/milliseconds delay.
    setdelay --gps                    Specify ticks/milliseconds delay.
    setdelay --matrix                 Specify ticks/milliseconds delay.
    setdelay --pcinput                Specify ticks/milliseconds delay.
    setdelay --log                    Specify ticks/milliseconds delay.

    example: setdelay --admplex0 1 --gyro0 1 --gps 1

-----

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
    stat --sentence --pcinput   Takes arguments -e, -d.
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

Other

    -v    Enable verbosoity.
    -vv   Enable extra verbosoity.
    help

-----

![plot](./Data/SatIO_Stat.png)

-----

  Wiring For Keystudio ESP32 PLUS Development Board:

          ESP32: 1st ATMEGA2560 with shield as Port Controller (not on multiplexer):
          ESP32: I2C SDA -> ATMEGA2560: I2C SDA
          ESP32: I2C SCL -> ATMEGA2560: I2C SCL

          Other ESP32 I2C Devices (not on multiplexer):
          ESP32: SDA0 SCL0 -> DS3231 (RTC): SDA, SCL (5v)

          ESP32 i2C: I2C Multiplexing (3.3v) (for peripherals):
          ESP32: i2C -> TCA9548A: SDA, SCL

          ESP32: Analog/Digital Multiplexing (3.3v) (for peripherals):
          ESP32: io4    -> CD74HC4067: SIG
          ESP32: io32   -> CD74HC4067: S0
          ESP32: io33   -> CD74HC4067: S1
          ESP32: io16   -> CD74HC4067: S2
          ESP32: io17   -> CD74HC4067: S3
          CD74HC4067 C0 -> DHT11: SIG

          ESP32: WTGPS300P (5v) (for getting a downlink):
          ESP32: io27 RXD -> WTGPS300P: TXD
          ESP32: null TXD -> WTGPS300P: RXD

          ESP32: WT901 9-Axis Gyro:
          ESP32: Serial2 RXD -> WT901 TXD
          ESP32: Serial2 TXD -> WT901 RXD
          
-----

    To Do: AI I2C modules returning int's as classifiers.
    To Do: SRTM data. Use NASA shuttle radar topographical mission data.
    To Do: Ability to add custom IIC sensor modules after flashing.
    To Do: PCB fabrication.
    To Do: UI after migration complete.

-----

  There are some required custom libs included in complete project files:
  https://drive.google.com/drive/folders/13yynSxkKL-zxb7iLSkg0v0VXkSLgmtW-?usp=sharing

-----




















