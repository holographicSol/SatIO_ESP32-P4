/*
  Serial Information Command Library.

  Returns information over serial.
  Commands system over serial. 
*/

#include "serial_infocmd.h"
#include <Arduino.h>
#include "matrix.h"
#include "wtgps300p.h"
#include "wt901.h"
#include "multiplexers.h"
#include "custommapping.h"
#include "sidereal_helper.h"
#include "satio.h"
#include "ins.h"
#include "meteors.h"
#include "hextodig.h"
#include "esp32_helper.h"
#include "sdmmc_helper.h"
#include "arg_parser.h"
#include "system_data.h"
#include <FS.h>
#include "SD_MMC.h"
#include "SPIFFS.h"
#include <stdlib.h>
#include "satio_file.h"
#include <limits.h>
#include <float.h>

ArgParser parser;
PlainArgParser plainparser;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 DATA: SERIAL 0
// ------------------------------------------------------------------------------------------------------------------------------
struct Serial0Struct serial0Data = {
  .nbytes=0, // number of bytes read by serial.
  .iter_token=0, // count token iterations.
  .BUFFER={}, // serial buffer.
  .token=0, // token pointer .
  .collected=0, // counts how many unique sentences have been collected.
  .checksum=0,
  .checksum_of_buffer=0,
  .checksum_in_buffer=0,
  .gotSum=0,
  .i_XOR=0,
  .XOR=0,
  .c_XOR=0,
};

// ----------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                         CmdProcess
// ----------------------------------------------------------------------------------------------------------------------------------

static void PrintHelp(void) {
  
  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                               SATIO                                               ");
  Serial.println("---------------------------------------------------------------------------------------------------");
  
  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                            INFORMATION (SENTENCES)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("switch output all                         Returns all following checksummed sentences.");
  Serial.println("switch output satio                       Return SatIO Sentence With Checksum.");
  Serial.println("switch output ins                         Return INS Sentence With Checksum.");
  Serial.println("switch output gngga                       Return GNGGA Sentence With Checksum.");
  Serial.println("switch output gnrmc                       Return GNRMC Sentence With Checksum.");
  Serial.println("switch output gpatt                       Return GPATT Sentence With Checksum.");
  Serial.println("switch output matrix                      Return Matrix Sentence With Checksum.");
  Serial.println("switch output admplex0                    Return Analog/Digital Multiplexer Sentence With Checksum.");
  Serial.println("switch output gyro0                       Return Gyro Sentence With Checksum.");
  Serial.println("switch output sun                         Return Sun Sentence With Checksum.");
  Serial.println("switch output moon                        Return Moon Sentence With Checksum.");
  Serial.println("switch output mercury                     Return Mercury Sentence With Checksum.");
  Serial.println("switch output venus                       Return Venus Sentence With Checksum.");
  Serial.println("switch output mars                        Return Mars Sentence With Checksum.");
  Serial.println("switch output jupiter                     Return Jupiter Sentence With Checksum.");
  Serial.println("switch output saturn                      Return Saturn Sentence With Checksum.");
  Serial.println("switch output uranus                      Return Uranus Sentence With Checksum.");
  Serial.println("switch output neptune                     Return Neptune Sentence With Checksum.");
  Serial.println("switch output meteors                     Return Meteors Sentence With Checksum.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                INFORMATION (SATIO)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print satio latitude                      Return SATIO Degrees Latitude.");
  Serial.println("print satio longitude                     Return SATIO Degrees Longitude.");
  Serial.println("print satio ground heading                Return SATIO Ground Heading.");
  Serial.println("print satio time                          Return SATIO Formatted Local Time.");
  Serial.println("print satio date                          Return SATIO Formatted Local Date.");
  Serial.println("print satio sync time                     Return SATIO Formatted RTC Sync Time.");
  Serial.println("print satio sync date                     Return SATIO Formatted RTC Sync Date.");
  Serial.println("print satio rtc time                      Return SATIO Formatted RTC Time.");
  Serial.println("print satio rtc date                      Return SATIO Formatted RTC Date.");
  Serial.println("print satio utc offset                    Return SATIO UTC Second Offset.");
  Serial.println("print satio utc auto offset               Return SATIO UTC Second Offset Flag.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                               INFORMATION (MATRIX)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print matrix -v                           Return all matrix data.");
  Serial.println("print matrix n                            Return matrix data for specified n.");
  Serial.println("print matrix functions                    Return verbose all available matrix switch functions.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                INFORMATION (GNGGA)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print gngga time                          Return GNGGA UTC Time.");
  Serial.println("print gngga latitude                      Return GNGGA Latitude.");
  Serial.println("print gngga latitude hemisphere           Return GNGGA Latitude Hemisphere.");
  Serial.println("print gngga longitude                     Return GNGGA Longitude.");
  Serial.println("print gngga longitude hemisphere          Return GNGGA Longitude Hemisphere.");
  Serial.println("print gngga solution status               Return GNGGA Solution Status.");
  Serial.println("print gngga satellite count               Return GNGGA Satellite Count.");
  Serial.println("print gngga gps precision factor          Return GNGGA HDOP Precision Factor.");
  Serial.println("print gngga altitude                      Return GNGGA Altitude.");
  Serial.println("print gngga altitude units                Return GNGGA Altitude Units.");
  Serial.println("print gngga geoidal                       Return GNGGA Geoidal.");
  Serial.println("print gngga geoidal units                 Return GNGGA Geoidal Units.");
  Serial.println("print gngga differential delay            Return GNGGA Differential Delay.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                INFORMATION (GNRMC)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print gnrmc time                          Return GNRMC UTC Time.");
  Serial.println("print gnrmc positioning status            Return GNRMC Positioning Status.");
  Serial.println("print gnrmc latitude                      Return GNRMC Latitude.");
  Serial.println("print gnrmc latitude hemisphere           Return GNRMC Latitude Hemisphere.");
  Serial.println("print gnrmc longitude                     Return GNRMC Longitude.");
  Serial.println("print gnrmc longitude hemisphere          Return GNRMC Longitude Hemisphere.");
  Serial.println("print gnrmc ground speed                  Return GNRMC Ground Speed.");
  Serial.println("print gnrmc ground heading                Return GNRMC Ground Heading.");
  Serial.println("print gnrmc date                          Return GNRMC UTC Date.");
  Serial.println("print gnrmc installation angle            Return GNRMC Installation Angle.");
  Serial.println("print gnrmc installation angle direction  Return GNRMC Installation Angle Direction.");
  Serial.println("print gnrmc mode indication               Return GNRMC Mode Indication.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                 INFORMATION(GPATT)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print gpatt pitch                         Return GPATT Pitch.");
  Serial.println("print gpatt roll                          Return GPATT Roll.");
  Serial.println("print gpatt yaw                           Return GPATT Yaw.");
  Serial.println("print gpatt software version              Return GPATT Software Version.");
  Serial.println("print gpatt product id                    Return GPATT Product ID.");
  Serial.println("print gpatt ins                           Return GPATT INS.");
  Serial.println("print gpatt hardware version              Return GPATT Hardware Version.");
  Serial.println("print gpatt run_state_flag                Return GPATT Run State Flag.");
  Serial.println("print gpatt mis angle num                 Return GPATT Mis Angle Num.");
  Serial.println("print gpatt static flag                   Return GPATT Static Flag.");
  Serial.println("print gpatt user code                     Return GPATT User Code.");
  Serial.println("print gpatt gst data                      Return GPATT GST Data.");
  Serial.println("print gpatt line flag                     Return GPATT Line Flag.");
  Serial.println("print gpatt mis att flag                  Return GPATT Mis Att Flag.");
  Serial.println("print gpatt imu kind                      Return GPATT IMU Kind.");
  Serial.println("print gpatt ubi car kind                  Return GPATT UBI Car Kind.");
  Serial.println("print gpatt mileage                       Return GPATT Mileage.");
  Serial.println("print gpatt run inetial flag              Return GPATT Run Inetial Flag.");
  Serial.println("print gpatt speed enable                  Return GPATT Speed Enable.");
  Serial.println("print gpatt speed num                     Return GPATT Speed Num.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                            INFORMATION (ADMPLEX 0)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print admplex0 n                          Return Analog/Digital Multiplexer 0 Channel n (0-15) Value.");
  
  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                INFORMATION (Gyro0)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print gyro0 ang x                         Return Gyro0 Angle X.");
  Serial.println("print gyro0 ang y                         Return Gyro0 Angle Y.");
  Serial.println("print gyro0 ang z                         Return Gyro0 Angle Z.");
  Serial.println("print gyro0 mag x                         Return Gyro0 Magnetic Field X.");
  Serial.println("print gyro0 mag y                         Return Gyro0 Magnetic Field Y.");
  Serial.println("print gyro0 mag z                         Return Gyro0 Magnetic Field Z.");
  Serial.println("print gyro0 acc x                         Return Gyro0 Acceleration X.");
  Serial.println("print gyro0 acc y                         Return Gyro0 Acceleration Y.");
  Serial.println("print gyro0 acc z                         Return Gyro0 Acceleration Z.");
  Serial.println("print gyro0 gyr x                         Return Gyro0 Gyro X.");
  Serial.println("print gyro0 gyr y                         Return Gyro0 Gyro Y.");
  Serial.println("print gyro0 gyr z                         Return Gyro0 Gyro Z.");

  Serial.println("");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                  INFORMATION (SUN)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print sun ra                              Return Sun Right Ascension.");
  Serial.println("print sun dec                             Return Sun Declination.");
  Serial.println("print sun az                              Return Sun Azimuth.");
  Serial.println("print sun alt                             Return Sun Altitude.");
  Serial.println("print sun r                               Return Sunrise.");
  Serial.println("print sun s                               Return Sunset.");
  Serial.println("print sun helat                           Return Sun Helioecliptic Latitude.");
  Serial.println("print sun helon                           Return Sun Helioecliptic Longitude.");
  Serial.println("print sun rv                              Return Sun Radius Vector.");
  Serial.println("print sun dis                             Return Sun Distance.");
  Serial.println("print sun elat                            Return Sun Ecliptic Latitude");
  Serial.println("print sun elon                            Return Sun Ecliptic Longitude.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                              INFORMATION (MERCURY)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print mercury ra                          Return Mercury Right Ascension.");
  Serial.println("print mercury dec                         Return Mercury Declination.");
  Serial.println("print mercury az                          Return Mercury Azimuth");
  Serial.println("print mercury alt                         Return Mercury Altitude.");
  Serial.println("print mercury r                           Return Mercury Rise.");
  Serial.println("print mercury s                           Return Mercury Set.");
  Serial.println("print mercury hlat                        Return Mercury Helioecliptic Latitude.");
  Serial.println("print mercury hlon                        Return Mercury Helioecliptic Longitude.");
  Serial.println("print mercury rv                          Return Mercury Radius Vector.");
  Serial.println("print mercury dis                         Return Mercury Distance.");
  Serial.println("print mercury elat                        Return Mercury Ecliptic Latitude.");
  Serial.println("print mercury elon                        Return Mercury Ecliptic Longitude.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                INFORMATION (VENUS)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print venus ra                            Return Venus Right Ascension.");
  Serial.println("print venus dec                           Return Venus Declination.");
  Serial.println("print venus az                            Return Venus Azimuth");
  Serial.println("print venus alt                           Return Venus Altitude.");
  Serial.println("print venus r                             Return Venus Rise.");
  Serial.println("print venus s                             Return Venus Set.");
  Serial.println("print venus hlat                          Return Venus Helioecliptic Latitude.");
  Serial.println("print venus hlon                          Return Venus Helioecliptic Longitude.");
  Serial.println("print venus rv                            Return Venus Radius Vector.");
  Serial.println("print venus dis                           Return Venus Distance.");
  Serial.println("print venus elat                          Return Venus Ecliptic Latitude.");
  Serial.println("print venus elon                          Return Venus Ecliptic Longitude.");

  Serial.println("");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                INFORMATION (EARTH)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print earth el                            Return Earth Ecliptic Longitude.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                 INFORMATION (MOON)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print moon ra                             Return Moon Right Ascension.");
  Serial.println("print moon dec                            Return Moon Declination.");
  Serial.println("print moon az                             Return Moon Azimuth.");
  Serial.println("print moon alt                            Return Moon Altitude.");
  Serial.println("print moon r                              Return Moon Rise.");
  Serial.println("print moon s                              Return Moon Set.");
  Serial.println("print moon p                              Return Moon Phase.");
  Serial.println("print moon l                              Return Moon Luminosity.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                 INFORMATION (MARS)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print mars ra                             Return Mars Right Ascension.");
  Serial.println("print mars dec                            Return Mars Declination.");
  Serial.println("print mars az                             Return Mars Azimuth");
  Serial.println("print mars alt                            Return Mars Altitude.");
  Serial.println("print mars r                              Return Mars Rise.");
  Serial.println("print mars s                              Return Mars Set.");
  Serial.println("print mars hlat                           Return Mars Helioecliptic Latitude.");
  Serial.println("print mars hlon                           Return Mars Helioecliptic Longitude.");
  Serial.println("print mars rv                             Return Mars Radius Vector.");
  Serial.println("print mars dis                            Return Mars Distance.");
  Serial.println("print mars elat                           Return Mars Ecliptic Latitude.");
  Serial.println("print mars elon                           Return Mars Ecliptic Longitude.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                              INFORMATION (JUPITER)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print jupiter ra                          Return Juptier Right Ascension.");
  Serial.println("print jupiter dec                         Return Juptier Declination.");
  Serial.println("print jupiter az                          Return Juptier Azimuth");
  Serial.println("print jupiter alt                         Return Juptier Altitude.");
  Serial.println("print jupiter r                           Return Juptier Rise.");
  Serial.println("print jupiter s                           Return Juptier Set.");
  Serial.println("print jupiter hlat                        Return Juptier Helioecliptic Latitude.");
  Serial.println("print jupiter hlon                        Return Juptier Helioecliptic Longitude.");
  Serial.println("print jupiter rv                          Return Juptier Radius Vector.");
  Serial.println("print jupiter dis                         Return Juptier Distance.");
  Serial.println("print jupiter elat                        Return Juptier Ecliptic Latitude.");
  Serial.println("print jupiter elon                        Return Juptier Ecliptic Longitude.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                               INFORMATION (SATURN)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print saturn ra                           Return Saturn Right Ascension.");
  Serial.println("print saturn dec                          Return Saturn Declination.");
  Serial.println("print saturn az                           Return Saturn Azimuth");
  Serial.println("print saturn alt                          Return Saturn Altitude.");
  Serial.println("print saturn r                            Return Saturn Rise.");
  Serial.println("print saturn s                            Return Saturn Set.");
  Serial.println("print saturn hlat                         Return Saturn Helioecliptic Latitude.");
  Serial.println("print saturn hlon                         Return Saturn Helioecliptic Longitude.");
  Serial.println("print saturn rv                           Return Saturn Radius Vector.");
  Serial.println("print saturn dis                          Return Saturn Distance.");
  Serial.println("print saturn elat                         Return Saturn Ecliptic Latitude.");
  Serial.println("print saturn elon                         Return Saturn Ecliptic Longitude.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                               INFORMATION (URANUS)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print uranus ra                           Return Uranus Right Ascension.");
  Serial.println("print uranus dec                          Return Uranus Declination.");
  Serial.println("print uranus az                           Return Uranus Azimuth");
  Serial.println("print uranus alt                          Return Uranus Altitude.");
  Serial.println("print uranus r                            Return Uranus Rise.");
  Serial.println("print uranus s                            Return Uranus Set.");
  Serial.println("print uranus hlat                         Return Uranus Helioecliptic Latitude.");
  Serial.println("print uranus hlon                         Return Uranus Helioecliptic Longitude.");
  Serial.println("print uranus rv                           Return Uranus Radius Vector.");
  Serial.println("print uranus dis                          Return Uranus Distance.");
  Serial.println("print uranus elat                         Return Uranus Ecliptic Latitude.");
  Serial.println("print uranus elon                         Return Uranus Ecliptic Longitude.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                              INFORMATION (NEPTUNE)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("print neptune ra                          Return Neptune Right Ascension.");
  Serial.println("print neptune dec                         Return Neptune Declination.");
  Serial.println("print neptune az                          Return Neptune Azimuth");
  Serial.println("print neptune alt                         Return Neptune Altitude.");
  Serial.println("print neptune r                           Return Neptune Rise.");
  Serial.println("print neptune s                           Return Neptune Set.");
  Serial.println("print neptune hlat                        Return Neptune Helioecliptic Latitude.");
  Serial.println("print neptune hlon                        Return Neptune Helioecliptic Longitude.");
  Serial.println("print neptune rv                          Return Neptune Radius Vector.");
  Serial.println("print neptune dis                         Return Neptune Distance.");
  Serial.println("print neptune elat                        Return Neptune Ecliptic Latitude.");
  Serial.println("print neptune elon                        Return Neptune Ecliptic Longitude.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                              INFORMATION (OBJECTS)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("indetify object n n                       Attempt to identify an object by RA n & DEC n.");
  
  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                INFORMATION (Bench)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("stat                                      Output system information continuously.");
  Serial.println("                                          stat. Simple output");
  Serial.println("                                          stat v. Increased verbosity");
  Serial.println("                                          stat vv. Increased verbosity (may require full screen).");
  Serial.println("stat sdcard                               Output sdcard information once.");
  Serial.println("print looptime max                        Return slowest recorded looptime since startup in microseconds.");
  Serial.println("print partition table                     Returns partition table.");
  Serial.println("print ram info                            Returns RAM info.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("[ CONTROL ] Requires Serial Commands enabled.");
  Serial.println("---------------------------------------------------------------------------------------------------");
  
  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                   CONTROL (MATRIX)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("new matrix                                Clears any current matrix value.");
  Serial.println();
  Serial.println("set output value n n                      Set matrix switch output value.");
  Serial.println("                                          Automatically overrides computer.");
  Serial.println("                                          To hand back control to the computer: set assist enabled n.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : output value n.");
  Serial.println();
  Serial.println("set assist enabled n                      Set computer assist enabled.");
  Serial.println("                                          0 : matrix n.");
  Serial.println();
  Serial.println("set assist disabled n                     Set computer assist disabled.");
  Serial.println("                                          0 : matrix n.");
  Serial.println();
  Serial.println("set matrix function n n n                 Set matrix function.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : function n.");
  Serial.println("                                          2 : available function n (see available functions).");
  Serial.println();
  Serial.println("set matrix x n n n                        Set matrix function value X.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : function n.");
  Serial.println("                                          2 : function value n.");
  Serial.println();
  Serial.println("set matrix y n n n                        Set matrix function value Y.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : function n.");
  Serial.println("                                          2 : function value n.");
  Serial.println();
  Serial.println("set matrix z n n n                        Set matrix function value Z.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : function n.");
  Serial.println("                                          2 : function value n.");
  Serial.println();
  Serial.println("set matrix port n n                       Set matrix output port number.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : output port n (-1=None).");
  Serial.println();
  Serial.println("set matrix inverted n n n                 Invert matrix function logic.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : function n.");
  Serial.println("                                          2 : standard/inverted n (0-1).");
  Serial.println();
  Serial.println("set matrix operator n n n                 Set matrix function operator.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : function n.");
  Serial.println("                                          2 : Set operator n. 0=None  1=Equal  2=Over  3=Under  4=Range");
  Serial.println();
  Serial.println("set matrix entry n n n x y z n n n        Set a matrix.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : function n.");
  Serial.println("                                          2 : function index n.");
  Serial.println("                                          3 : function values x.");
  Serial.println("                                          4 : function values y.");
  Serial.println("                                          5 : function values z.");
  Serial.println("                                          6 : standard/inverted n (0-1).");
  Serial.println("                                          7 : operator n (0-4). 0=None  1=Equal  2=Over  3=Under  4=Range");
  Serial.println("                                          8 : output port n (-1=None).");
  Serial.println();
  Serial.println("set mapping n v emin emax omin omax       Start mapping a value.");
  Serial.println("                                          Useful for stabalizing a fluctuating value to a specified range.");
  Serial.println("                                          Can be used in other ways.");
  Serial.println("                                          0 : mapping slot n (0-69 slots).");
  Serial.println("                                          1 : value index n.");
  Serial.println("                                          2 : expected value min.");
  Serial.println("                                          3 : expected value max.");
  Serial.println("                                          4 : output value min.");
  Serial.println("                                          5 : output value max.");
  Serial.println();
  Serial.println("set output map n n                        Set mapped value to use as output.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : mapping slot n.");
  Serial.println();
  Serial.println("set output mode n n                       Set switch output mode.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : mode n (0: default, 1: mapped value).");
  Serial.println();
  Serial.println("set flux n n                              Set a fluctuation threshold.");
  Serial.println("                                          Useful for stabalizing a fluctuating value that needs to remain full ADC bit range.");
  Serial.println("                                          Can be used in other ways.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : flux n.");
  Serial.println();
  Serial.println("set map mode n n                          Set mapping algorithm to use.");
  Serial.println("                                          0 : matrix n.");
  Serial.println("                                          1 : mode n (0: min to max, 1: center map x0, 2: center map x1).");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                      CONTROL (GPS)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("set convert coordinates GNGGA             Set Coordinate Degrees Conversions From GNGGA Data.");
  Serial.println("set convert coordinates GNRMC             Set Coordinate Degrees Conversions From GNRMC Data.");

  Serial.println("set speed units n                           Set speed units.");
  Serial.println("                                            0 : meters per second (M/S).");
  Serial.println("                                            1 : miles per hour (MPH).");
  Serial.println("                                            2 : kilometers per hour (KPH).");
  Serial.println("                                            3 : knots (KTS).");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                 CONTROL (DATETIME)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("set utc offset                            Set UTC offset in seconds.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                     CONTROL (GYRO)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("set speed units n                           Set speed units.");
  Serial.println("gyro0 cal acc                               Calibrate acceleration to current orientation.");
  Serial.println("gyro0 cal mag start                         Start magnetic field calibration.");
  Serial.println("gyro0 cal mag end                           End magnetic field calibration.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                      CONTROL (INS)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("set ins mode n                              Set INS mode n.");
  Serial.println("                                            0 : Do not use INS");
  Serial.println("                                            1 : Dynamic. Estimate position between primary GPS sets.");
  Serial.println("                                            2 : Hold The Line. Do not allow GPS to set INS until further notice.");
  Serial.println("reset ins forced flag                       Manually reset special flag set in INS mode 2.");
  Serial.println("set ins heading gyro                        Always use gyro heading for INS heading.");
  Serial.println("                                            Gyro must be properly calibrated.");
  Serial.println("set ins heading gps                         Always use GPS heading for INS heading.");
  Serial.println("                                            Only recommended if INS does not require changes in heading while estimating.");

  Serial.println("set ins gps precision n                     Set required minimum GPS precision for INS.");
  Serial.println("set ins min speed n                         Set required minimum speed for INS.");
  Serial.println("set ins heading range diff n                Set required max range difference between gyro heading and GPS heading.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                  CONTROL (STORAGE)");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("unmount sdcard                              Attempt to unmount sdcard.");
  Serial.println("mount sdcard                                Attempt to mount sdcard.");

  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                                                                               HELP");
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("h                                         Display this help message.");
  Serial.println("");
  Serial.println("---------------------------------------------------------------------------------------------------");
}

void setAllSentenceOutput(bool enable) {
  systemData.output_satio_enabled=enable;
  systemData.output_gngga_enabled=enable;
  systemData.output_gnrmc_enabled=enable;
  systemData.output_gpatt_enabled=enable;
  systemData.output_ins_enabled=enable;
  systemData.output_matrix_enabled=enable;
  systemData.output_admplex0_enabled=enable;
  systemData.output_gyro_0_enabled=enable;
  systemData.output_sun_enabled=enable;
  systemData.output_moon_enabled=enable;
  systemData.output_mercury_enabled=enable;
  systemData.output_venus_enabled=enable;
  systemData.output_mars_enabled=enable;
  systemData.output_jupiter_enabled=enable;
  systemData.output_saturn_enabled=enable;
  systemData.output_uranus_enabled=enable;
  systemData.output_neptune_enabled=enable;
  systemData.output_meteors_enabled=enable;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GET CHECKSUM
// ------------------------------------------------------------------------------------------------------------------------------

int getCheckSumSerial0(char * string) {
  for (serial0Data.XOR=0, serial0Data.i_XOR=0; serial0Data.i_XOR < strlen(string); serial0Data.i_XOR++) {
    serial0Data.c_XOR=(unsigned char)string[serial0Data.i_XOR];
    if (serial0Data.c_XOR=='*') break;
    if (serial0Data.c_XOR != '$') serial0Data.XOR ^= serial0Data.c_XOR;
  }
  return serial0Data.XOR;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                              VALIDATE CHECKSUM
// ------------------------------------------------------------------------------------------------------------------------------

bool validateChecksumSerial0(char * buffer) {
  memset(serial0Data.gotSum, 0, sizeof(serial0Data.gotSum));
  serial0Data.gotSum[0]=buffer[strlen(buffer) - 3];
  serial0Data.gotSum[1]=buffer[strlen(buffer) - 2];
  serial0Data.checksum_of_buffer= getCheckSumSerial0(buffer);
  serial0Data.checksum_in_buffer=h2d2(serial0Data.gotSum[0], serial0Data.gotSum[1]);
  if (serial0Data.checksum_in_buffer==serial0Data.checksum_of_buffer) {return true;}
  return false;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                CREATE CHECKSUM
// ------------------------------------------------------------------------------------------------------------------------------

void createChecksumSerial0(char * buffer) {
  serial0Data.checksum_of_buffer=getCheckSumSerial0(buffer);
  sprintf(serial0Data.checksum,"%X",serial0Data.checksum_of_buffer);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     VALIDATION
// ------------------------------------------------------------------------------------------------------------------------------

bool val_global_element_size(const char * data) {
  if (sizeof(data)>=MAX_GLOBAL_ELEMENT_SIZE) {return false;}
  return true;
}

bool val_switch_index(const char * data) {
  if (str_is_int8(data)) {if (atol(data)<MAX_MATRIX_SWITCHES) {return true;}}
  return false;
}

bool val_function_index(const char * data) {
  if (str_is_int8(data)) {if (atol(data)<MAX_MATRIX_SWITCH_FUNCTIONS) {return true;}}
  return false;
}

bool val_speed_units(const char * data) {
  if (str_is_int8(data)) {if (atol(data)<MAX_SPEED_CONVERSION_MODES) {return true;}}
  return false;
}

bool val_function_name_index(const char * data) {
  if (val_global_element_size(data)==false) {return false;}
  if (str_is_long(data)!=true) {return false;}
  if (atol(data) < MAX_MATRIX_FUNCTION_NAMES) {return true;}
  return false;
}

bool val_function_xyz(const char * data) {
  if (str_is_double(data)) {return true;}
  return false;
}

bool val_function_operator(const char * data) {
  if (str_is_int8(data)) {return true;}
  return false;
}

bool val_switch_port(const char * data) {
  if (str_is_int8(data)) {return true;}
  return false;
}

bool val_mappable_value_index(const char * data) {
  Serial.println(data);
  if (str_is_int8(data)!=true) {return false;}
  if (atoi(data)<MAX_MAPPABLE_VALUES) {return true;} 
  return false;
}

bool val_ins_mode(const char * data) {
  if (str_is_int8(data)) {if (atol(data)<MAX_INS_MODE) {return true;}}
  return false;
}

bool val_ins_gps_precision(const char * data) {
  if (str_is_float(data)) {return true;}
  return false;
}

bool val_ins_minimum_speed(const char * data) {
  if (str_is_float(data)) {return true;}
  return false;
}

bool val_ins_heading_range_diff(const char * data) {
  if (str_is_float(data)) {return true;}
  return false;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                            PRINT SYSTEM CONFIG
// ------------------------------------------------------------------------------------------------------------------------------
void PrintSystemData(void) {
    Serial.println("-----------------------------------------------------");
    Serial.println("[System] ");
    Serial.println("[serial_command] " + String(systemData.serial_command));
    Serial.println("[output_satio_all] " + String(systemData.output_satio_all));
    Serial.println("[output_satio_enabled] " + String(systemData.output_satio_enabled));
    Serial.println("[output_ins_enabled] " + String(systemData.output_ins_enabled));
    Serial.println("[output_gngga_enabled] " + String(systemData.output_gngga_enabled));
    Serial.println("[output_gnrmc_enabled] " + String(systemData.output_gnrmc_enabled));
    Serial.println("[output_gpatt_enabled] " + String(systemData.output_gpatt_enabled));
    Serial.println("[output_matrix_enabled] " + String(systemData.output_matrix_enabled));
    Serial.println("[output_admplex0_enabled] " + String(systemData.output_admplex0_enabled));
    Serial.println("[output_gyro_0_enabled] " + String(systemData.output_gyro_0_enabled));
    Serial.println("[output_sun_enabled] " + String(systemData.output_sun_enabled));
    Serial.println("[output_moon_enabled] " + String(systemData.output_moon_enabled));
    Serial.println("[output_mercury_enabled] " + String(systemData.output_mercury_enabled));
    Serial.println("[output_venus_enabled] " + String(systemData.output_venus_enabled));
    Serial.println("[output_mars_enabled] " + String(systemData.output_mars_enabled));
    Serial.println("[output_jupiter_enabled] " + String(systemData.output_jupiter_enabled));
    Serial.println("[output_saturn_enabled] " + String(systemData.output_saturn_enabled));
    Serial.println("[output_uranus_enabled] " + String(systemData.output_uranus_enabled));
    Serial.println("[output_neptune_enabled] " + String(systemData.output_neptune_enabled));
    Serial.println("[output_meteors_enabled] " + String(systemData.output_meteors_enabled));
    Serial.println("-----------------------------------------------------");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                            PRINT SYSTEM CONFIG
// ------------------------------------------------------------------------------------------------------------------------------
void PrintSatIOData(void) {
    Serial.println("-----------------------------------------------------");
    Serial.println("[SatIO] ");
    Serial.println("[coordinate_conversion_mode] " + String(satioData.char_coordinate_conversion_mode[satioData.coordinate_conversion_mode]));
    Serial.println("[speed_conversion_mode] " + String(satioData.char_speed_conversion_mode[satioData.speed_conversion_mode]));
    Serial.println("[utc_second_offset] " + String(satioData.utc_second_offset));
    Serial.println("[utc_auto_offset_flag] " + String(satioData.utc_auto_offset_flag));
    Serial.println("[set_time_automatically] " + String(satioData.set_time_automatically));
    Serial.println("-----------------------------------------------------");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             PRINT MAPPING DATA
// ------------------------------------------------------------------------------------------------------------------------------
void PrintMappingData(void) {
  for (int Mi=0; Mi<MAX_MAP_SLOTS; Mi++) {
    Serial.println("-----------------------------------------------------");
    Serial.println("[slot] " + String(Mi));
    Serial.println("[map_mode] " + String(mappingData.map_mode[0][Mi]));
    Serial.println("[map slot idx] " + String(mappingData.index_mapped_value[0][Mi]));
    Serial.println("[map config 0] " + String(mappingData.mapping_config[0][Mi][0]));
    Serial.println("[map config 1] " + String(mappingData.mapping_config[0][Mi][1]));
    Serial.println("[map config 2] " + String(mappingData.mapping_config[0][Mi][2]));
    Serial.println("[map config 3] " + String(mappingData.mapping_config[0][Mi][3]));
    Serial.println("[map config 4] " + String(mappingData.mapping_config[0][Mi][4]));
    Serial.println("[map config 5] " + String(mappingData.mapping_config[0][Mi][5]));
    Serial.println("-----------------------------------------------------");
  }
}

void PrintMatrixConfig() {
    Serial.println("-----------------------------------------------------");
    Serial.println("[load_matrix_on_startup] " + String(matrixData.load_matrix_on_startup));
    Serial.println("[Available Switch Functions]");
    for (int Mi=0; Mi<MAX_MATRIX_FUNCTION_NAMES; Mi++) {Serial.println("  [" + String(Mi) + "] " + String(matrixData.matrix_function_names[Mi]));}
    Serial.println("[Available Switch Function Operators]");
    for (int Mi=0; Mi<MAX_MATRIX_OPERATORS; Mi++) {Serial.println("  [" + String(Mi) + "] " + String(matrixData.matrix_function_operator_name[Mi]));}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   PRINT MATRIX
// ------------------------------------------------------------------------------------------------------------------------------
void PrintMatrixData(void) {
    for (int Mi=0; Mi<MAX_MATRIX_SWITCHES; Mi++) {
    Serial.println("-----------------------------------------------------");
    Serial.println("[matrix switch] " + String(Mi));
    Serial.println("[computer assist] " + String(matrixData.computer_assist[0][Mi]));
    Serial.println("[output mode] " + String(matrixData.output_mode[0][Mi]));
    Serial.println("[map slot] " + String(mappingData.index_mapped_value[0][Mi]));
    Serial.println("[flux] " + String(matrixData.flux_value[0][Mi]));
    Serial.println("[pwm] 0: " + String(matrixData.output_pwm[0][Mi][0]) + " 1: " + String(matrixData.output_pwm[0][Mi][1]));
    Serial.println("[port] " + String(matrixData.matrix_port_map[0][Mi]));
    Serial.println("[active] " + String(matrixData.switch_intention[0][Mi]));
    for (int Fi=0; Fi<MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
      Serial.println("[function " + String(Fi) + " name] " + String(matrixData.matrix_function_names[matrixData.matrix_function[0][Mi][Fi]]));
      Serial.println("[function " + String(Fi) + " matrix_function_operator_name] " + String(matrixData.matrix_switch_operator_index[0][Mi][Fi]));
      Serial.println("[function " + String(Fi) + " inverted] " + String(matrixData.matrix_switch_inverted_logic[0][Mi][Fi]));
      Serial.println("[function " + String(Fi) + " x] " + String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]));
      Serial.println("[function " + String(Fi) + " y] " + String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]));
      Serial.println("[function " + String(Fi) + " z] " + String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]));
    }
  }
  Serial.println("-----------------------------------------------------");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 PRINT MATRIX N
// ------------------------------------------------------------------------------------------------------------------------------
void PrintMatrixNData() {
  int Mi=atoi(plainparser.tokens[0]);
  Serial.println("-----------------------------------------------------");
  Serial.println("[matrix switch] " + String(Mi));
  Serial.println("[computer assist] " + String(matrixData.computer_assist[0][Mi]));
  Serial.println("[output mode] " + String(matrixData.output_mode[0][Mi]));
  Serial.println("[flux] " + String(matrixData.flux_value[0][Mi]));
  Serial.println("[pwm] 0: " + String(matrixData.output_pwm[0][Mi][0]) + " 1: " + String(matrixData.output_pwm[0][Mi][1]));
  Serial.println("[port] " + String(matrixData.matrix_port_map[0][Mi]));
  Serial.println("[active] " + String(matrixData.switch_intention[0][Mi]));
  for (int Fi=0; Fi<MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
    Serial.println("[function " + String(Fi) + " name] " + String(matrixData.matrix_function_names[matrixData.matrix_function[0][Mi][Fi]]));
    Serial.println("[function " + String(Fi) + " matrix_function_operator_name] " + String(matrixData.matrix_switch_operator_index[0][Mi][Fi]));
    Serial.println("[function " + String(Fi) + " inverted] " + String(matrixData.matrix_switch_inverted_logic[0][Mi][Fi]));
    Serial.println("[function " + String(Fi) + " x] " + String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]));
    Serial.println("[function " + String(Fi) + " y] " + String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]));
    Serial.println("[function " + String(Fi) + " z] " + String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]));
  }
  Serial.println("-----------------------------------------------------");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                          PRINT AVAILABLE MATRIX FUNCTION NAMES
// ------------------------------------------------------------------------------------------------------------------------------
void PrintAvailableMatrixFunctionNames() {
  Serial.println("[Available Matrix Functions] ");
  for (int i=0; i<MAX_MATRIX_FUNCTION_NAMES; i++) {Serial.println("[" + String(i) + "] " + String(matrixData.matrix_function_names[i]));}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     SET MATRIX
// ------------------------------------------------------------------------------------------------------------------------------

void setMatrixPort(int switch_idx, signed int port_n) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && port_n>=-1 && port_n<MAX_MATRIX_SWITCHES) {
    matrixData.matrix_port_map[0][switch_idx]=port_n;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixFunction(int switch_idx, int func_idx, int func_n) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_n>=0 && func_n<MAX_MATRIX_FUNCTION_NAMES) {
    matrixData.matrix_function[0][switch_idx][func_idx]=func_n;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixX(int switch_idx, int func_idx, double func_x) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_x>=DBL_MIN && func_x<DBL_MAX) {
    matrixData.matrix_function_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_X]=func_x;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixY(int switch_idx, int func_idx, double func_y) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_y>=DBL_MIN && func_y<DBL_MAX) {
    matrixData.matrix_function_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_Y]=func_y;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixZ(int switch_idx, int func_idx, double func_z) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_z>=DBL_MIN && func_z<DBL_MAX) {
    matrixData.matrix_function_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_Z]=func_z;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixInverted(int switch_idx, int func_idx, int func_i) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_i>=0 && func_i<=1) {
    matrixData.matrix_switch_inverted_logic[0][switch_idx][func_idx]=func_i;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixOperator(int switch_idx, int func_idx, int func_o) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_o>=0 && func_o<MAX_MATRIX_OPERATORS) {
    matrixData.matrix_switch_operator_index[0][switch_idx][func_idx]=func_o;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixModulation(int switch_idx, uint32_t pwm0, uint32_t pwm1) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && pwm0>=0 && pwm0<UINT32_MAX && pwm1>=0 && pwm1<UINT32_MAX) {
    matrixData.output_pwm[0][switch_idx][0]=pwm0;
    matrixData.output_pwm[0][switch_idx][1]=pwm1;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setFlux(int switch_idx, uint32_t flux) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && flux>=0 && flux<LONG_MAX) {
    matrixData.flux_value[0][switch_idx]=flux;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setOutputMode(int switch_idx, int output_mode) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && output_mode>=0 && output_mode<MAX_MATRIX_OUTPUT_MODES) {
    matrixData.output_mode[0][switch_idx]=output_mode;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setOverrideOutputValue(int switch_idx, uint32_t override_value) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && override_value>=LONG_MIN && override_value<LONG_MAX) {
    matrixData.computer_assist[0][switch_idx]=false;
    matrixData.override_output_value[0][switch_idx]=override_value;
    long i_retry;
    while (matrixData.computer_assist[0][switch_idx]!=false) {
      matrixData.computer_assist[0][switch_idx]=false;
      i_retry++;
      if (i_retry==MAX_MATRIX_OVERRIDE_TIME) {Serial.println("WARNING! Could not override computer_assist!"); break;}
      delayMicroseconds(1);
    }
    i_retry=0;
    while (matrixData.override_output_value[0][switch_idx]!=override_value) {
      matrixData.override_output_value[0][switch_idx]=override_value;
      i_retry++;
      if (i_retry==MAX_MATRIX_OVERRIDE_TIME) {Serial.println("WARNING! Could not override override_output_value!"); break;}
      delayMicroseconds(1);
    }
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

char *cmd_proc_xyzptr;

void setComputerAssist(int switch_idx, bool computer_assist) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && computer_assist>=0 && computer_assist<=1) {
    matrixData.computer_assist[0][switch_idx]=computer_assist;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setINSMode(int ins_mode) {
  if (ins_mode>=0 && ins_mode <MAX_INS_MODE) {insData.INS_MODE=ins_mode;}
}

void setINSGPSPrecision(double ins_precision) {
  if (ins_precision>=0 && ins_precision<DBL_MAX) {insData.INS_REQ_GPS_PRECISION=ins_precision;}
}

void setINSMinSpeed(double ins_min_speed) {
  if (ins_min_speed>=0 && ins_min_speed<DBL_MAX) {insData.INS_REQ_MIN_SPEED=ins_min_speed;}
}

void setINSHeadingRangeDiff(double ins_range_diff) {
  if (ins_range_diff>=0 && ins_range_diff<DBL_MAX) {insData.INS_REQ_HEADING_RANGE_DIFF=ins_range_diff;}
}

void setSpeedUnits(int speed_units) {
  if (speed_units>=0 && speed_units <MAX_SPEED_CONVERSION_MODES) {satioData.speed_conversion_mode=speed_units;}
}

void setUTCSecondOffset(int64_t seconds) {
  if (seconds>=LONG_LONG_MIN && seconds<LONG_LONG_MAX) {satioData.utc_second_offset=seconds;}
}

void setMapConfig(int map_slot, int map_mode, signed long c0, signed long c1,signed long c2,signed long c3, signed long c4,
                signed long c5) {
  if (map_slot>=0 && map_slot<MAX_MAP_SLOTS && map_mode>=0 && map_mode<MAX_MAP_MODES && c0>=INT32_MIN && c0<INT32_MAX
      && c1>=INT32_MIN && c1<INT32_MAX && c2>=INT32_MIN && c2<INT32_MAX && c3>=LONG_MIN && c3<INT32_MAX
      && c4>=INT32_MIN && c4<INT32_MAX && c5>=INT32_MIN && c5<INT32_MAX) {
    mappingData.mapping_config[0][map_slot][0]=c0;
    mappingData.mapping_config[0][map_slot][1]=c1;
    mappingData.mapping_config[0][map_slot][2]=c2;
    mappingData.mapping_config[0][map_slot][3]=c3;
    mappingData.mapping_config[0][map_slot][4]=c4;
    mappingData.mapping_config[0][map_slot][5]=c5;
    mappingData.map_mode[0][map_slot]=map_mode;
    matrixData.matrix_switch_write_required[0][map_slot]=true;
  }
}

void setMapSlot(int matrix_switch, int map_slot) {
  if (matrix_switch>=0 && matrix_switch<MAX_MAP_SLOTS && map_slot>=0 && map_slot<=1) {
    mappingData.index_mapped_value[0][matrix_switch]=map_slot;
    matrixData.matrix_switch_write_required[0][map_slot]=true;
  }
}

void saveMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_SLOTS) {
    memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath));
    strcpy(satioFileData.current_matrix_filepath, satioFileData.matix_filepaths[matrix_file_slot]);
    sdmmcFlagData.save_matrix=true;
  }
}

void loadMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_SLOTS) {
    memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath));
    strcpy(satioFileData.current_matrix_filepath, satioFileData.matix_filepaths[matrix_file_slot]);
    sdmmcFlagData.load_matrix=true;
  }
}

void deleteMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_SLOTS) {
    memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath));
    strcpy(satioFileData.current_matrix_filepath, satioFileData.matix_filepaths[matrix_file_slot]);
    sdmmcFlagData.delete_matrix=true;
  }
}

// ----------------------------------------------------------------------------------------------------------------------------------
/*
  StarNav.
*/
// ----------------------------------------------------------------------------------------------------------------------------------
void star_nav() {
  // star sirius test: starnav 6 45 8.9 -16 42 58.0
  // ngc test:         starnav 2 20 35.0 -23 7 0.0
  // ic test:          starnav 17 46 18 5 43 00
  // other obj test:   starnav 1 36 0 61 17 0
  // messier test:     starnav 16 41 40 36 28 0
  // caldwel test:     starnav 1 19 32.6 58 17 27
  // Herschel test:    starnav 0 29 56.0 60 14 0.0
  // new stars test    starnav 0 02 07.2 -14 40 34
  // caldwel test:     starnav 00 13 0 72 32 0
  // non-h400          starnav 7 38 3 -14 52 3
  //                   starnav 1 33 9 30 39 0
  simple_argparser_init_from_buffer(&plainparser, serial0Data.BUFFER, 1);
  if ((str_is_int32(plainparser.tokens[0])==true) &&
      (str_is_int32(plainparser.tokens[1])==true) &&
      (str_is_float(plainparser.tokens[2])==true) &&
      (str_is_int32(plainparser.tokens[3])==true) &&
      (str_is_int32(plainparser.tokens[4])==true) &&
      (str_is_float(plainparser.tokens[5])==true)
    )
  {
    Serial.println("attempting to identify object..");
    // this is identify (so first identify object)
    IdentifyObject(
      atoi(plainparser.tokens[0]),
      atoi(plainparser.tokens[1]),
      atof(plainparser.tokens[2]),
      atoi(plainparser.tokens[3]),
      atoi(plainparser.tokens[4]),
      atof(plainparser.tokens[5])
    );
    //
    /*
      Once identified we can track object (requires modified SiderealObjects lib).
    */
    trackObject(satioData.degrees_latitude, satioData.degrees_longitude,
      satioData.rtc_year, satioData.rtc_month, satioData.rtc_mday,
      satioData.rtc_hour, satioData.rtc_minute, satioData.rtc_second,
      satioData.local_hour, satioData.local_minute, satioData.local_second,
      atof(gnggaData.altitude), siderealObjectData.object_table_i, siderealObjectData.object_number);
    Serial.println("---------------------------------------------");
    Serial.println("Table Index:   " + String(siderealObjectData.object_table_i));
    Serial.println("Table:         " + String(siderealObjectData.object_table_name));
    Serial.println("Number:        " + String(siderealObjectData.object_number));
    Serial.println("Name:          " + String(siderealObjectData.object_name));
    Serial.println("Type:          " + String(siderealObjectData.object_type));
    Serial.println("Constellation: " + String(siderealObjectData.object_con));
    Serial.println("Distance:      " + String(siderealObjectData.object_dist));
    Serial.println("Azimuth:       " + String(siderealObjectData.object_az));
    Serial.println("Altitude:      " + String(siderealObjectData.object_alt));
    Serial.println("Rise:          " + String(siderealObjectData.object_r));
    Serial.println("Set:           " + String(siderealObjectData.object_s));
    // magnitude from earth
    // distance from earth minus altitude+location over earth
    // magnitude from earth minus altitude+location over earth
    Serial.println("---------------------------------------------");
  }
  else {Serial.println("identify object: bad input data");}
}

// ----------------------------------------------------------------------------------------------------------------------------------
/*
  SDCard.
*/
// ----------------------------------------------------------------------------------------------------------------------------------
void unmountSDCard() {
  sdmmcFlagData.no_delay_flag=true;
  sdmmcFlagData.unmount_sdcard_flag=true;
}

void mountSDCard() {
  sdmmcFlagData.no_delay_flag=true;
  sdmmcFlagData.mount_sdcard_flag=true;
}

// ----------------------------------------------------------------------------------------------------------------------------------
/*
  Debug ArgParse.
  Expected behaviour:
  command: foo -a -b -c
  flags:   a b c
  command: foo -a 1 -b 2 -c 3
  flags:   a="1" b="2" c="3"
  Note:
    - For best practice only use ArgParser if flags are required, else use PlainArgParser for simple tokenization.
    - Use PlainArgParser if processing negative numbers.
    - short flags: 1-3 alphanumeric chars. example: -a, -a1, -a12, -abc.
    - long flags: 1-256 alphanumeric chars. example: --foobar, --foo-bar, --foobar123.
    - see ArgParser for more details.
*/
// ----------------------------------------------------------------------------------------------------------------------------------
size_t pos_count;
const char** pos;
bool verbose;
bool verbose_1;
bool enable;

void printArgParse() {
  Serial.println("-------------------------------------------");
  Serial.print("[debug] First command: ");
  if (pos_count > 0) {Serial.println(pos[0]);}
  else {Serial.println("none");}
  Serial.print("[debug] Positionals (");
  Serial.print(pos_count);
  Serial.print("): ");
  for (size_t j = 0; j < pos_count; ++j) {Serial.print(pos[j]); if (j < pos_count - 1) Serial.print(" ");}
  Serial.println();
  Serial.println("----");
  Serial.print("[debug] Flag count: ");
  Serial.println(parser.flag_count);
  Serial.print("[debug] Flags: ");
  for (size_t k = 0; k < parser.flag_count; ++k) {Serial.print(parser.flags[k]); const char* val = parser.values[k];
      if (val[0] != '\0') {Serial.print("=\""); Serial.print(val); Serial.print("\"");}
      if (k < parser.flag_count - 1) Serial.print(" ");
  }
  Serial.println();
  Serial.println("-------------------------------------------");
}

// ----------------------------------------------------------------------------------------------------------------------------------
/*
  Command Process.
  The following commands are intended to allow SatIO to be controlled via other systems, embedded systems, scripts and humans.
  This function is being upgraded to use arg_parser.
*/
// ----------------------------------------------------------------------------------------------------------------------------------
void CmdProcess(void) {
  memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
  while (Serial.available()) {Serial.readBytesUntil('\n', serial0Data.BUFFER, sizeof(serial0Data.BUFFER)-1);}
  // --------------------------------------------------
  // process commands conditionally (some efficiency).
  // --------------------------------------------------
  if (strlen(serial0Data.BUFFER)>=2) {
    // ------------------------------------------------
    // Debug Serial Buffer.
    // ------------------------------------------------
    Serial.println("[CmdProcess] " + String(serial0Data.BUFFER));
    // --------------------------------------------------
    // Initialize argparse.
    // --------------------------------------------------
    argparser_reset(&parser);
    if (!argparser_init_from_buffer(&parser, serial0Data.BUFFER)) {fprintf(stderr, "[cmd] Failed to initialize parser from buffer\n"); return;}
    pos_count=0;
    pos={};
    pos = argparser_get_positionals(&parser, &pos_count);
    // --------------------------
    // Verbosity.
    // --------------------------
    verbose=false;
    verbose_1=false;
    verbose = argparser_get_bool(&parser, "v") || argparser_get_bool(&parser, "verbose");
    verbose_1 = argparser_get_bool(&parser, "vv") || argparser_get_bool(&parser, "verbose1");
    if (verbose_1) {verbose=true;}
    if (verbose==false) {verbose_1=false;}
    Serial.println("[cmd] verbose: " + String(verbose));
    Serial.println("[cmd] verbose1: " + String(verbose_1));

    // --------------------------
    // Debug Arg Parse.
    // --------------------------
    printArgParse();
    // --------------------------------------------------
    // Commands.
    // --------------------------------------------------
    if (strcmp(pos[0], "help")==0 || strcmp(pos[0], "h")==0) {printf("Usage: [buffer with] [--flag value] [-f value] [positional...]\n");
      if (verbose) {PrintHelp();}
    }
    else if (strcmp(pos[0], "stat")==0) {
      enable=false;
      if (argparser_has_flag(&parser, "disable") || argparser_has_flag(&parser, "d")) {enable=false;}
      else if (argparser_has_flag(&parser, "enable") || argparser_has_flag(&parser, "e")) {enable=true;}
      if (argparser_has_flag(&parser, "t")) {
        if (enable) {systemData.output_stat=enable; systemData.output_stat_v=verbose; systemData.output_stat_vv=verbose_1;}
        else {systemData.output_stat=false; systemData.output_stat_v=false; systemData.output_stat_vv=false;}
      }
      if (argparser_has_flag(&parser, "partition-table")) {print_partition_table();}
      if (argparser_has_flag(&parser, "memory-ram")) {print_ram_info();}
      if (argparser_has_flag(&parser, "sdcard")) {statSDCard();}
      if (strcmp(serial0Data.BUFFER, "stat --system")==0) {PrintSystemData();}
      if (strcmp(serial0Data.BUFFER, "stat --matrix\r")==0) {PrintMatrixConfig();}
      if (strncmp(serial0Data.BUFFER, "stat --matrix ", strlen("stat --matrix "))==0) {
        if (argparser_has_flag(&parser, "A")) {PrintMatrixData();}
        else {simple_argparser_init_from_buffer(&plainparser, serial0Data.BUFFER, 2); if (val_switch_index(plainparser.tokens[0])) {PrintMatrixNData();}}
      }
      else if (strncmp(serial0Data.BUFFER, "stat --mapping", strlen("stat --mapping"))==0) {PrintMappingData();}
      else if (argparser_has_flag(&parser, "sentence")) {
        if (argparser_has_flag(&parser, "A")) {systemData.output_satio_all=enable; setAllSentenceOutput(enable);}
        if (argparser_has_flag(&parser, "satio")) {systemData.output_satio_enabled=enable;}
        if (argparser_has_flag(&parser, "gngga")) {systemData.output_gngga_enabled=enable;}
        if (argparser_has_flag(&parser, "gnrmc")) {systemData.output_gnrmc_enabled=enable;}
        if (argparser_has_flag(&parser, "gpatt")) {systemData.output_gpatt_enabled=enable;}
        if (argparser_has_flag(&parser, "ins")) {systemData.output_ins_enabled=enable;}
        if (argparser_has_flag(&parser, "matrix")) {systemData.output_matrix_enabled=enable;}
        if (argparser_has_flag(&parser, "admplex0")) {systemData.output_admplex0_enabled=enable;}
        if (argparser_has_flag(&parser, "gyro0")) {systemData.output_gyro_0_enabled=enable;}
        if (argparser_has_flag(&parser, "sun")) {systemData.output_sun_enabled=enable;}
        if (argparser_has_flag(&parser, "moon")) {systemData.output_moon_enabled=enable;}
        if (argparser_has_flag(&parser, "mercury")) {systemData.output_mercury_enabled=enable;}
        if (argparser_has_flag(&parser, "venus")) {systemData.output_venus_enabled=enable;}
        if (argparser_has_flag(&parser, "mars")) {systemData.output_mars_enabled=enable;}
        if (argparser_has_flag(&parser, "jupiter")) {systemData.output_jupiter_enabled=enable;}
        if (argparser_has_flag(&parser, "saturn")) {systemData.output_saturn_enabled=enable;}
        if (argparser_has_flag(&parser, "uranus")) {systemData.output_uranus_enabled=enable;}
        if (argparser_has_flag(&parser, "neptune")) {systemData.output_neptune_enabled=enable;}
        if (argparser_has_flag(&parser, "meteors")) {systemData.output_meteors_enabled=enable;}
      }
    }
    
    else if (strcmp(pos[0], "ls")==0) {
      const char* path_str = argparser_get_path(&parser, "/");
      Serial.println("[cmd] Path: " + String(path_str));
      int maxlevels=0;
      if (argparser_has_flag(&parser, "R")) {maxlevels=-1;}
      else {maxlevels = argparser_get_int8(&parser, "maxlevels", 1);}
      memset(sdmmcArgData.buffer, 0, sizeof(sdmmcArgData.buffer));
      strcpy(sdmmcArgData.buffer, path_str);
      sdmmcArgData.maxlevels=maxlevels;
      sdmmcFlagData.no_delay_flag=true;
      sdmmcFlagData.list_dir_flag=true;
    }

    else if (strcmp(pos[0], "starnav")==0) {star_nav();}
    
    // --------------------------------------------------
    //  "Restricted Commands".
    // --------------------------------------------------
    if (systemData.serial_command) {
      if (strcmp(pos[0], "system")==0) {
        if (argparser_has_flag(&parser, "save")) {sdmmcFlagData.save_system=true;}
        else if (argparser_has_flag(&parser, "load")) {sdmmcFlagData.load_system=true;}
        else if (argparser_has_flag(&parser, "restore-defaults")) {restore_system_defaults();}
      }
      else if (strcmp(pos[0], "mapping")==0) {
        if (argparser_has_flag(&parser, "new")) {zero_mapping(); return;}
        else if (argparser_has_flag(&parser, "save")) {sdmmcFlagData.save_mapping=true;}
        else if (argparser_has_flag(&parser, "load")) {sdmmcFlagData.load_mapping=true;}
        else if (argparser_has_flag(&parser, "delete")) {sdmmcFlagData.delete_mapping=true;}
        else {
          int s  = argparser_get_int8(&parser, "s", -1);
          if (s==-1) {return;}
          setMapConfig(s, argparser_get_int8(&parser, "m", mappingData.map_mode[0][s]),
                       argparser_get_int32(&parser, "c0", mappingData.mapping_config[0][s][0]),
                       argparser_get_int32(&parser, "c1", mappingData.mapping_config[0][s][1]),
                       argparser_get_int32(&parser, "c2", mappingData.mapping_config[0][s][2]),
                       argparser_get_int32(&parser, "c3", mappingData.mapping_config[0][s][3]),
                       argparser_get_int32(&parser, "c4", mappingData.mapping_config[0][s][4]),
                       argparser_get_int32(&parser, "c5", mappingData.mapping_config[0][s][5]));}
      }
      else if (strcmp(pos[0], "matrix")==0) {
        if (argparser_has_flag(&parser, "startup-enable")) {matrixData.load_matrix_on_startup=true;}
        else if (argparser_has_flag(&parser, "startup-disable")) {matrixData.load_matrix_on_startup=false;}
        else if (argparser_has_flag(&parser, "new")) {zero_matrix(); return;}
        else if (argparser_has_flag(&parser, "save")) {saveMatrix(argparser_get_int8(&parser, "save", -1));}
        else if (argparser_has_flag(&parser, "load")) {loadMatrix(argparser_get_int8(&parser, "load", -1));}
        else if (argparser_has_flag(&parser, "delete")) {deleteMatrix(argparser_get_int8(&parser, "delete", -1));}
        else {
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "p")) {
            setMatrixPort(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "p", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fn")) {
            setMatrixFunction(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", -1), argparser_get_int8(&parser, "fn", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fx")) {
            setMatrixX(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_double(&parser, "fx", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fy")) {
            setMatrixY(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_double(&parser, "fy", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fz")) {
            setMatrixZ(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_double(&parser, "fz", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fi")) {
            setMatrixInverted(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_bool(&parser, "fi"));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fo")) {
            setMatrixOperator(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_int8(&parser, "fo", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "pwm0") && argparser_has_flag(&parser, "pwm1")) {
            setMatrixModulation(argparser_get_int8(&parser, "s", -1), argparser_get_uint32(&parser, "pwm0", 0), argparser_get_uint32(&parser, "pwm1", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "flux")) {
            setFlux(argparser_get_int8(&parser, "s", -1), argparser_get_uint32(&parser, "flux", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "oride")) {
            setOverrideOutputValue(argparser_get_int8(&parser, "s", -1), argparser_get_int32(&parser, "oride", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "computer-assist")) {
            setComputerAssist(argparser_get_int8(&parser, "s", -1), argparser_get_bool(&parser, "computer-assist"));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "omode")) {
            setOutputMode(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "omode", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "map-slot")) {
            setMapSlot(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "map-slot", -1));
          }
        }
      }
      else if (strcmp(pos[0], "ins")==0) {
        if (argparser_has_flag(&parser, "m")) {setINSMode(argparser_get_int8(&parser, "m", INS_MODE_DYNAMIC));}
        if (argparser_has_flag(&parser, "gyro")) {insData.INS_USE_GYRO_HEADING=argparser_get_bool(&parser, "gyro");}
        if (argparser_has_flag(&parser, "p")) {setINSGPSPrecision(argparser_get_double(&parser, "p", 0.5));}
        if (argparser_has_flag(&parser, "s")) {setINSMinSpeed(argparser_get_double(&parser, "s", 0.3));}
        if (argparser_has_flag(&parser, "r")) {setINSHeadingRangeDiff(argparser_get_double(&parser, "r", 0));}
        if (argparser_has_flag(&parser, "reset-forced")) {insData.INS_FORCED_ON_FLAG=false;}
      }
      else if (strcmp(pos[0], "satio")==0) {
        if (argparser_has_flag(&parser, "speed-units")) {setSpeedUnits(argparser_get_int8(&parser, "speed-units", 0));}
        if (argparser_has_flag(&parser, "utc-offset")) {setUTCSecondOffset(argparser_get_int64(&parser, "utc-offset", 0));}
        if (argparser_has_flag(&parser, "mode-gngga")) {satioData.coordinate_conversion_mode=0;}
        else if (argparser_has_flag(&parser, "mode-gnrmc")) {satioData.coordinate_conversion_mode=1;}
      }
      else if (strcmp(pos[0], "gyro")==0) {
        if (argparser_has_flag(&parser, "calacc")) {WT901CalAcc();}
        if (argparser_has_flag(&parser, "calmag-start")) {WT901CalMagStart();}
        else if (argparser_has_flag(&parser, "calmag-stop")) {WT901CalMagEnd();}
      }
      else if (strcmp(pos[0], "sdcard")==0) {
        if (argparser_has_flag(&parser, "mount")) {mountSDCard();}
        else if (argparser_has_flag(&parser, "unmount")) {unmountSDCard();}
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               OUTPUT SENTENCES
// ------------------------------------------------------------------------------------------------------------------------------
// output sentences proceedurally so that all sentence output should be clean and on a new line.
// not recommended to be used alongside individual serial print arguments; example print satio degrees_latitude.
// ------------------------------------------------------------------------------------------------------------------------------

void outputSentences(void) {

  // Serial.println("outputSentences");

  if (systemData.interval_breach_1_second==true) {
    outputStat();
  }

  if (systemData.interval_breach_gps) {
    systemData.interval_breach_gps=0;
    if (systemData.output_gngga_enabled) {Serial.println(gnggaData.outsentence);}
    if (systemData.output_gnrmc_enabled) {Serial.println(gnrmcData.outsentence);}
    if (systemData.output_gpatt_enabled) {Serial.println(gpattData.outsentence);}
    if (systemData.output_satio_enabled) {
      // -----------------------------
      // start building satio sentence
      // -----------------------------
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$SATIO,");
      // -----------------------------
      // rtc time
      // -----------------------------
      strcat(serial0Data.BUFFER, String(satioData.padded_rtc_time_HHMMSS).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -----------------------------
      // rtc date
      // -----------------------------
      strcat(serial0Data.BUFFER, String(satioData.padded_rtc_date_DDMMYYYY).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -------------------
      // rtc sync time
      // -------------------
      strcat(serial0Data.BUFFER, String(satioData.padded_rtc_sync_time_HHMMSS).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -------------------
      // rtc sync date
      // -------------------
      strcat(serial0Data.BUFFER, String(satioData.padded_rtc_sync_date_DDMMYYYY).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -------------------
      // local time
      // -------------------
      strcat(serial0Data.BUFFER, String(satioData.padded_local_time_HHMMSS).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -------------------
      // local date
      // -------------------
      strcat(serial0Data.BUFFER, String(satioData.padded_local_date_DDMMYYYY).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -----------------------------
      // system uptime in seconds
      // -----------------------------
      strcat(serial0Data.BUFFER, String(systemData.uptime_seconds).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -----------------------------
      // latitude degrees
      // -----------------------------
      strcat(serial0Data.BUFFER, String(satioData.degrees_latitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -----------------------------
      // longitude degrees
      // -----------------------------
      strcat(serial0Data.BUFFER, String(satioData.degrees_longitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -----------------------------
      // append checksum
      // -----------------------------
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      // -----------------------------
      // output
      // -----------------------------
      Serial.println(serial0Data.BUFFER);
      // Serial.println(String(i_count_read_gps) + " : " + String(serial0Data.BUFFER));
    }
  }

  if (systemData.interval_breach_ins) {
    systemData.interval_breach_ins = 0;
    if (systemData.output_ins_enabled) {
      // -----------------------------
      // start building satio sentence
      // -----------------------------
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$INS,");
      strcat(serial0Data.BUFFER, String(satioData.rtc_unixtime).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.INS_INITIALIZATION_FLAG).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_latitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_longitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_altitude).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_heading).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_speed).c_str());
      strcat(serial0Data.BUFFER, ",");
      // -----------------------------
      // append checksum
      // -----------------------------
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      // -----------------------------
      // output
      // -----------------------------
      Serial.println(serial0Data.BUFFER);
    }
  }

  if (systemData.interval_breach_gyro_0) {
    systemData.interval_breach_gyro_0 = 0;
    if (systemData.output_gyro_0_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$GYRO0,");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_acc_x).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_acc_y).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_acc_z).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_ang_x).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_ang_y).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_ang_z).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_gyr_x).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_gyr_y).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_gyr_z).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_mag_x).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_mag_y).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_mag_z).c_str());
      strcat(serial0Data.BUFFER, ",");
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      // output
      Serial.println(serial0Data.BUFFER);
      // Serial.println(String(i_count_read_gyro_0) + " : " + String(serial0Data.BUFFER));
    }
  }

  if (systemData.interval_breach_track_planets) {
    systemData.interval_breach_track_planets = 0;
    if (systemData.output_sun_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$SUN,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_s + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }

    if (systemData.output_moon_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MOON,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_p + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_lum + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }

    if (systemData.output_mercury_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MERCURY,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_ecliptic_long + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }

    if (systemData.output_venus_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$VENUS,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_ecliptic_long + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
      // Serial.println(String(i_count_track_planets) + " : " + String(serial0Data.BUFFER));
    }

    if (systemData.output_mars_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MARS,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_ecliptic_long + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }


    if (systemData.output_jupiter_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$JUPITER,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_ecliptic_long + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }

    if (systemData.output_saturn_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$SATURN,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_ecliptic_long + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }

    if (systemData.output_uranus_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$URANUS,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_ecliptic_long + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }

    if (systemData.output_neptune_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$NEPTUNE,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_ecliptic_long + String(",")).c_str());
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }

    if (systemData.output_meteors_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$METEOR,");
      for (int i=0; i<MAX_METEOR_SHOWERS; i++) {
        // in datetime range
        strcat(serial0Data.BUFFER, String(String(meteor_shower_warning_system[i][0]) + String(",")).c_str());
        // in peak datetiem range
        strcat(serial0Data.BUFFER, String(String(meteor_shower_warning_system[i][1]) + String(",")).c_str());
      }
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
  }

  // if (systemData.interval_breach_matrix) {
    // systemData.interval_breach_matrix = 0;
    if (systemData.output_matrix_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcpy(serial0Data.BUFFER, "$MATRIX,");
      // append port mapping data (optional at a significant performance cost)
      // for (int i=0; i < MAX_MATRIX_SWITCHES; i++) {strcat(serial0Data.BUFFER, String(String(matrixData.matrix_port_map[0][i])+",").c_str());}
      // append matrix switch state data
      for (int i=0; i < MAX_MATRIX_SWITCHES; i++) {strcat(serial0Data.BUFFER, String(String(matrixData.switch_intention[0][i])+",").c_str());}
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
      // Serial.println(String(i_count_matrix) + " : " + String(serial0Data.BUFFER));
    }
  // }

  if (systemData.interval_breach_mplex) {
    systemData.interval_breach_mplex = 0;
    if (systemData.output_admplex0_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MPLEX0,");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[0]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[1]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[2]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[3]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[4]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[5]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[6]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[7]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[8]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[9]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[10]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[11]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[12]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[13]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[14]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[15]).c_str());
      strcat(serial0Data.BUFFER, ",");
      // append checksum
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      // output
      Serial.println(serial0Data.BUFFER);
      // Serial.println(String(i_count_read_mplex) + " : " + String(serial0Data.BUFFER));
    }
  }
}

void printArray(signed long arr[], int start, int end) {
    for (int i = start; i < end; i++) {
        printf("%-7d", arr[i]); // Left-align with 5-character width
        // if ((i + 1) % 5 == 0)  // New line after every 5 elements
        //     printf("\n");
    }
    printf("\n");
}

// uncomment to use
signed long print_index_0[35]={0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
signed long print_index_1[35]={35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69};
char counter_chars_0[15][56]={"Loops p/s", "GPS p/s", "INS p/s", "Gyro0 p/s", "ADMplex0 p/s", "CMD p/s", "Universe p/s", "Matrix p/s", "I/O p/s", "LT uS", "LT Max uS", "Satellites", "GPS Precision", };
double counter_digits_0[15]={};

char counter_chars_1_row_0[9][56]={"Time", "Date", "UNIX Time", "Latitude", "Longitude", "Altitude", "Heading", "Speed", "Mileage"};
char counter_chars_1_col_0[5][56]={"GPS", "RTC", "RTC Sync", "System", "System INS"};
char counter_digits_1_row_N[9][56]={};

void outputStat(void) {
    // ---------------------------------------------------------------------
    //                                                 EXECUTIONS PER SECOND
    // ---------------------------------------------------------------------
    // note that output will not be suitable at too large a font/zoom
    // ---------------------------------------------------------------------
    // System Stat Overview.
    // column width supports numbers of up to 9,999,999.
    // All matrix xyz digits are rounded to fit column width.
    // Column width is 7 (million+space).
    // Any chars/digits wider than 6 chars will overlap/touch next column.
    // For development & diagnostic purposes.
    // ---------------------------------------------------------------------
    if (systemData.output_stat==true || systemData.output_stat_v==true || systemData.output_stat_vv) {
    // printAllTimes();
    // ----------------------------------------------------------------------------------------------------------------------------
    //                                                                                                               PRINT COUNTERS
    // ----------------------------------------------------------------------------------------------------------------------------
    Serial.println();
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    // printf("taskGyro", "Unused stack: %u words\n", watermark_task_gyro);
    // printf("taskUniverse", "Unused stack: %u words\n", watermark_task_universe);
    // printf("taskSwitches", "Unused stack: %u words\n", watermark_task_switches);
    // printf("taskGPS", "Unused stack: %u words\n", watermark_task_gps);
    // printf("taskMultiplexers", "Unused stack: %u words\n", watermark_task_multiplexers);
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    counter_digits_0[0]=systemData.total_loops_a_second;
    counter_digits_0[1]=systemData.total_gps;
    counter_digits_0[2]=systemData.total_ins;
    counter_digits_0[3]=systemData.total_gyro;
    counter_digits_0[4]=systemData.total_mplex;
    counter_digits_0[5]=systemData.total_infocmd;
    counter_digits_0[6]=systemData.total_universe;
    counter_digits_0[7]=systemData.total_matrix;
    counter_digits_0[8]=systemData.total_portcon;
    counter_digits_0[9]=systemData.mainLoopTimeTaken;
    counter_digits_0[10]=systemData.mainLoopTimeTakenMax;
    counter_digits_0[11]=atoi(gnggaData.satellite_count);
    counter_digits_0[12]=atof(gnggaData.gps_precision_factor);
    for (int i = 0; i < 13; i++) {printf("%-16s", counter_chars_0[i]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    for (int i = 0; i < 13; i++) {printf("%-16f", counter_digits_0[i]);}
    printf("\n");
    Serial.println();
    // ----------------------------------------------------------------------------------------------------------------------------
    //                                                                                                           PRINT PRIMARY DATA
    // ----------------------------------------------------------------------------------------------------------------------------
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                   ");
    for (int i = 0; i < 9; i++) {printf("%-19s", counter_chars_1_row_0[i]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");

    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gnrmcData.utc_time).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gnrmcData.utc_date).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); // null
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); strcpy(counter_digits_1_row_N[3], String(gnrmcData.latitude).c_str());
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); strcpy(counter_digits_1_row_N[4], String(gnrmcData.longitude).c_str());
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(gnggaData.altitude, 7).c_str());
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); strcpy(counter_digits_1_row_N[6], String(gnrmcData.ground_heading, 7).c_str());
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); strcpy(counter_digits_1_row_N[7], String(gnrmcData.ground_speed, 7).c_str());
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); strcpy(counter_digits_1_row_N[8], String(gpattData.mileage, 7).c_str());
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");
    
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(satioData.padded_rtc_time_HHMMSS).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(satioData.padded_rtc_date_DDMMYYYY).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(satioData.rtc_unixtime).c_str());
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); // null
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); // null
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); // null
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); // null
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); // null
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); // null
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+1]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");

    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(satioData.padded_rtc_sync_time_HHMMSS).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(satioData.padded_rtc_sync_date_DDMMYYYY).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(satioData.rtcsync_unixtime).c_str());
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); strcpy(counter_digits_1_row_N[3], String(satioData.rtcsync_latitude).c_str());
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); strcpy(counter_digits_1_row_N[4], String(satioData.rtcsync_longitude).c_str());
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(satioData.rtcsync_altitude).c_str());
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); // null
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); // null
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); // null
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+2]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");

    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(satioData.padded_local_time_HHMMSS).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(satioData.padded_local_date_DDMMYYYY).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(satioData.local_unixtime_uS).c_str());
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); strcpy(counter_digits_1_row_N[3], String(satioData.degrees_latitude, 7).c_str());
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); strcpy(counter_digits_1_row_N[4], String(satioData.degrees_longitude, 7).c_str());
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(satioData.altitude).c_str()); // altitude: use srtm for meters above terrain elevation level (make other conversions also)
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); strcpy(counter_digits_1_row_N[6], String(satioData.ground_heading).c_str());
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); strcpy(counter_digits_1_row_N[7], String(String(satioData.speed, 2) + " " + String(satioData.char_speed_conversion_mode[satioData.speed_conversion_mode])).c_str()); // speed (3 dimensional: requires lat, long, alt, microtime)
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); strcpy(counter_digits_1_row_N[8], String(satioData.mileage).c_str()); // make mileage any 3d direction, refactor 'distance'
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+3]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");
    // utc offset
    // gforce.
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(satioData.padded_local_time_HHMMSS).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(satioData.padded_local_date_DDMMYYYY).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(satioData.local_unixtime_uS).c_str());
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); strcpy(counter_digits_1_row_N[3], String(insData.ins_latitude, 7).c_str());
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); strcpy(counter_digits_1_row_N[4], String(insData.ins_longitude, 7).c_str());
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(insData.ins_altitude).c_str());
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); strcpy(counter_digits_1_row_N[6], String(insData.ins_heading).c_str());
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); strcpy(counter_digits_1_row_N[7], String(String(insData.ins_speed, 2) + " " + String(satioData.char_speed_conversion_mode[satioData.speed_conversion_mode])).c_str());
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); strcpy(counter_digits_1_row_N[8], String("pending").c_str());
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+4]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("INS MODE : " + String(insData.INS_MODE) + " (" + String(insData.char_ins_mode[insData.INS_MODE]) + ") | INS FLAG : " + String(insData.INS_INITIALIZATION_FLAG) + "/" + String(MAX_INS_INITIALIZATION_FLAG) + " | INS FORCED ON FLAG : " + String(insData.INS_FORCED_ON_FLAG)
    + " | INS_REQ_GPS_PRECISION : " + String(insData.INS_REQ_GPS_PRECISION) + " | INS_REQ_MIN_SPEED : " + String(insData.INS_REQ_MIN_SPEED) + " | INS_REQ_HEADING_RANGE_DIFF : " + String(insData.INS_REQ_HEADING_RANGE_DIFF));
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("                   X                  Y                  Z");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_ang_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_ang_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_ang_z).c_str());
    Serial.print("Angle              ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_gyr_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_gyr_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_gyr_z).c_str());
    Serial.print("Gyro               ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_acc_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_acc_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_acc_z).c_str());
    Serial.print("Acceleration       ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_mag_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_mag_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_mag_z).c_str());
    Serial.print("Magnetic Field     ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("Weekday int: " + String(satioData.local_wday) + " Weekday Name: " + String(satioData.local_wday_name));
    Serial.print("Meteor Warning: ");
    for (int i = 0; i < MAX_METEOR_SHOWERS; i++) {Serial.print(String(String(meteor_shower_names[i]) + ": " + String(meteor_shower_warning_system[i][0]) + " " + String(meteor_shower_warning_system[i][1]) + " | ").c_str());}
    Serial.println();
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("SDCard mounted: " + String(sdcardData.sdcard_mounted) + " (" + String(sdcardData.sdcard_type_names[sdcardData.sdcard_type]) + " (type: " + String(sdcardData.sdcard_type) + ")");
  }
    // ----------------------------------------------------------------------------------------------------------------------------
    //                                                                                                      PRINT PROGRAMMABLE DATA
    // ----------------------------------------------------------------------------------------------------------------------------
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                      ");
    printArray(print_index_0, 0, 35);
    Serial.print("Computer Assist    :  ");
    for (int i=0;i<35;i++) {Serial.print(String(matrixData.computer_assist[0][i]) + "      ");}
    Serial.println();
    Serial.print("Switch Intention   :  ");
    for (int i=0;i<35;i++) {Serial.print(String(matrixData.switch_intention[0][i]) + "      ");}
    Serial.println();
    Serial.print("Computer Intention :  ");
    for (int i=0;i<35;i++) {Serial.print(String(matrixData.computer_intention[0][i]) + "      ");}
    Serial.println();
    Serial.print("Output Value       :  ");
    printArray(matrixData.output_value[0], 0, 35);
    }
    if (systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 0  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][0]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 1  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][1]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 2  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][2]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 3  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][3]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 4  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][4]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 0  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][5]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 6  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][6]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 7  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][7]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 8  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][8]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 9  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][9]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    }
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                      ");
    printArray(print_index_1, 0, 35);
    Serial.print("Computer Assist    :  ");
    for (int i=35;i<70;i++) {Serial.print(String(matrixData.computer_assist[0][i]) + "      ");}
    Serial.println();
    Serial.print("Switch Intention   :  ");
    for (int i=35;i<70;i++) {Serial.print(String(matrixData.switch_intention[0][i]) + "      ");}
    Serial.println();
    Serial.print("Computer Intention :  ");
    for (int i=35;i<70;i++) {Serial.print(String(matrixData.computer_intention[0][i]) + "      ");}
    Serial.println();
    Serial.print("Output Value       :  ");
    printArray(matrixData.output_value[0], 35, 70);
    }
    if (systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 0  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][0]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 1  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][1]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 2  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][2]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 3  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][3]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 4  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][4]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 0  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][5]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 6  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][6]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 7  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][7]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 8  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][8]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 9  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][9]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    }
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                      ");
    printArray(print_index_0, 0, 35);
    Serial.print("Mapped Values      :  ");
    printArray(mappingData.mapped_value[0], 0, 35);
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                      ");
    printArray(print_index_1, 0, 35);
    Serial.print("Mapped Values      :  ");
    printArray(mappingData.mapped_value[0], 35, 70);
    }
}