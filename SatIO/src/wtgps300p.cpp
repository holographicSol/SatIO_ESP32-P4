/*
    WTGPS300P Library. Written by Benjamin Jack Cullen.

    1 : From main call readGPS().
    2 : From main call validateGPSData().
    3 : All wtgps300p sentence data will now be available in:
        - gnggaData.
        - gnrmcData.
        - gpattData. 
*/

#include "wtgps300p.h"
#include <Arduino.h>
#include "hextodig.h"

// ----------------------------------------------------------------------------------------
// Global Data.
// ----------------------------------------------------------------------------------------
struct Serial1DataStruct serial1Data = {
    .nbytes = 0,
    .iter_token = 0,
    .BUFFER = {0},
    .token = NULL,
    .collected = 0,
    .gngga_bool = false,
    .gnrmc_bool = false,
    .gpatt_bool = false,
    .checksum = {0},
    .checksum_of_buffer = 0,
    .checksum_in_buffer = 0,
    .gotSum = {0},
    .i_XOR = 0,
    .XOR = 0,
    .c_XOR = 0
};

struct GNGGAStruct gnggaData = {
    .sentence = {0},
    .outsentence = {0},
    .tag = {0},
    .utc_time = {0},
    .latitude = {0},
    .latitude_hemisphere = {0},
    .longitude = {0},
    .longitude_hemisphere = {0},
    .solution_status = {0},
    .satellite_count = "0",
    .gps_precision_factor = {0},
    .altitude = {0},
    .altitude_units = {0},
    .geoidal = {0},
    .geoidal_units = {0},
    .differential_delay = {0},
    .base_station_id = {0},
    .check_sum = {0},
    .max_bad = ULONG_MAX - 2,
    .bad_element_bool = {0},
    .bad_element_count = {0},
    .element_name = {
        "Tag", "UTC Time", "Latitude", "Latitude Hemisphere", "Longitude",
        "Longitude Hemisphere", "Solution Status", "Satellite Count",
        "GPS Precision Factor", "Altitude", "Altitude Unit", "Geoidal",
        "Geoidal Unit", "Differential Delay", "Base Station ID", "Checksum"
    },
    .valid_checksum = false,
    .total_bad_elements = 0
};

int gngga_total_bad_elements = 0;

struct GNRMCStruct gnrmcData = {
    .sentence = {0},
    .outsentence = {0},
    .tag = {0},
    .utc_time = {0},
    .positioning_status = {0},
    .latitude = {0},
    .latitude_hemisphere = {0},
    .longitude = {0},
    .longitude_hemisphere = {0},
    .ground_speed = {0},
    .ground_heading = {0},
    .utc_date = {0},
    .installation_angle = {0},
    .installation_angle_direction = {0},
    .mode_indication = {0},
    .check_sum = {0},
    .max_bad = ULONG_MAX,
    .bad_element_bool = {0},
    .bad_element_count = {0},
    .element_name = {
        "Tag", "UTC Time", "Positioning Status", "Latitude", "Latitude Hemisphere",
        "Longitude", "Longitude Hemisphere", "Ground Speed", "Ground Heading",
        "UTC Date", "Installation Angle", "Installation Direction", "Mode Indication",
        "Checksum"
    },
    .valid_checksum = false,
    .total_bad_elements = 0
};

int gnrmc_total_bad_elements = 0;

struct GPATTStruct gpattData = {
    .sentence = {0},
    .outsentence = {0},
    .tag = {0},
    .pitch = {0},
    .angle_channel_0 = {0},
    .roll = {0},
    .angle_channel_1 = {0},
    .yaw = {0},
    .angle_channel_2 = {0},
    .software_version = {0},
    .version_channel = {0},
    .product_id = {0},
    .id_channel = {0},
    .ins = {0},
    .ins_channel = {0},
    .hardware_version = {0},
    .run_state_flag = {0},
    .mis_angle_num = {0},
    .custom_logo_0 = {0},
    .custom_logo_1 = {0},
    .custom_logo_2 = {0},
    .static_flag = {0},
    .user_code = {0},
    .gst_data = {0},
    .line_flag = {0},
    .custom_logo_3 = {0},
    .mis_att_flag = {0},
    .imu_kind = {0},
    .ubi_car_kind = {0},
    .mileage = {0},
    .custom_logo_4 = {0},
    .custom_logo_5 = {0},
    .run_inetial_flag = {0},
    .custom_logo_6 = {0},
    .custom_logo_7 = {0},
    .custom_logo_8 = {0},
    .custom_logo_9 = {0},
    .speed_enable = {0},
    .custom_logo_10 = {0},
    .custom_logo_11 = {0},
    .speed_num = {0},
    .scalable = {0},
    .check_sum = {0},
    .element_values = {{0}},
    .max_bad = ULONG_MAX,
    .bad_element_bool = {0},
    .bad_element_count = {0},
    .element_name = {
        "Tag", "Pitch", "Angle Channel 0", "Roll", "Angle Channel 1",
        "Yaw", "Angle Channel 2", "Software Version", "Version Channel",
        "Product ID", "ID Channel", "INS", "INS Channel", "Hardware Version",
        "Run State Flag", "MisAngle Num", "Custom Logo 0", "Custom Logo 1",
        "Custom Logo 2", "Static Flag", "User Code", "GST Data", "Line Flag",
        "Custom Logo 3", "MisAtt Flag", "IMU Kind", "UBI Car Kind", "Mileage",
        "Custom Logo 4", "Custom Logo 5", "Run Inertial Flag", "Custom Logo 6",
        "Custom Logo 7", "Custom Logo 8", "Custom Logo 9", "Speed Enable",
        "Custom Logo 10", "Custom Logo 11", "Speed Num", "Scalable", "Checksum"
    },
    .valid_checksum = false,
    .total_bad_elements = 0
};

int gpatt_total_bad_elements = 0;

// ----------------------------------------------------------------------------------------
// Validation Functions.
// ----------------------------------------------------------------------------------------
bool val_utc_time(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 9) return false;
    if (!isdigit(data[0]) || !isdigit(data[1]) || !isdigit(data[2]) || !isdigit(data[3]) ||
        !isdigit(data[4]) || !isdigit(data[5]) || data[6] != '.' ||
        !isdigit(data[7]) || !isdigit(data[8])) return false;
    return true;
}

bool val_utc_date(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 6) return false;
    if (!isdigit(data[0]) || !isdigit(data[1]) || !isdigit(data[2]) ||
        !isdigit(data[3]) || !isdigit(data[4]) || !isdigit(data[5])) return false;
    return true;
}

bool val_latitude(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_longitude(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_latitude_H(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'N' || data[0] == 'S');
}

bool val_longitude_H(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'E' || data[0] == 'W');
}

bool val_positioning_status_gngga(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == '0' || data[0] == '1' || data[0] == '2' || data[0] == '6');
}

bool val_satellite_count(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_long(data);
}

bool val_gps_precision_factor(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_altitude(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_altitude_units(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return true;
}

bool val_geoidal(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_geoidal_units(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return true;
}

bool val_differential_delay(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_basestation_id(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_long(data);
}

bool val_positioning_status_gnrmc(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'A' || data[0] == 'V');
}

bool val_ground_speed(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_ground_heading(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_installation_angle(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_installation_angle_direction(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'E' || data[0] == 'W');
}

bool val_mode_indication(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'A' || data[0] == 'D' || data[0] == 'E' || data[0] == 'N');
}

bool val_pitch_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_roll_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_yaw_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_angle_channle_p_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'p');
}

bool val_angle_channle_r_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'r');
}

bool val_angle_channle_y_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'y');
}

bool val_software_version_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_long(data);
}

bool val_version_channel_s_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == 'S');
}

bool val_product_id_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return is_alnum(data);
}

bool val_id_channel_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 2) return false;
    return (strcmp(data, "ID") == 0);
}

bool val_ins_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == '0' || data[0] == '1');
}

bool val_ins_channel_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 3) return false;
    return (strcmp(data, "INS") == 0);
}

bool val_hardware_version_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return true;
}

bool val_run_state_flag_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 2) return false;
    return (strcmp(data, "00") == 0 || strcmp(data, "01") == 0 ||
            strcmp(data, "02") == 0 || strcmp(data, "03") == 0 ||
            strcmp(data, "04") == 0);
}

bool val_mis_angle_num_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_long(data);
}

bool val_static_flag_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 2) return false;
    return (strcmp(data, "00") == 0 || strcmp(data, "01") == 0);
}

bool val_user_code_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_gst_data_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_long(data);
}

bool val_line_flag_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == '0' || data[0] == '1');
}

bool val_mis_att_flag_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 2) return false;
    return (strcmp(data, "00") == 0 || strcmp(data, "01") == 0);
}

bool val_imu_kind_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == '0' || data[0] == '1' || data[0] == '2' ||
            data[0] == '7' || data[0] == '8');
}

bool val_ubi_car_kind_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == '1' || data[0] == '2' || data[0] == '4');
}

bool val_mileage_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_double(data);
}

bool val_run_inetial_flag_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 2) return false;
    return (strcmp(data, "00") == 0 || strcmp(data, "01") == 0 ||
            strcmp(data, "02") == 0 || strcmp(data, "03") == 0 ||
            strcmp(data, "04") == 0);
}

bool val_speed_enable_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    if (strlen(data) != 1) return false;
    return (data[0] == '0' || data[0] == '1');
}

bool val_speed_num_gpatt(const char *data) {
    if (!val_element_size(data)) return false;
    return str_is_long(data);
}

bool val_custom_flag(const char *data) {
    if (!val_element_size(data)) return false;
    return true;
}

bool val_checksum(const char *data) {
    if (!val_element_size(data)) return false;
    return true;
}

bool val_element_size(const char *data) {
    if (data == NULL || strlen(data) >= MAX_GLOBAL_ELEMENT_SIZE) return false;
    return true;
}

// ----------------------------------------------------------------------------------------
// GNGGA Parsing.
// ----------------------------------------------------------------------------------------
void GNGGA(void) {
    serial1Data.iter_token = 0;
    serial1Data.token = strtok(gnggaData.sentence, ",");
    while (serial1Data.token != NULL) {
        switch (serial1Data.iter_token) {
            case 0:
                if (strcmp(serial1Data.token, "$GNGGA") == 0) {
                    memset(gnggaData.tag, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.tag, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 1:
                if (val_utc_time(serial1Data.token)) {
                    memset(gnggaData.utc_time, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.utc_time, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 2:
                if (val_latitude(serial1Data.token)) {
                    memset(gnggaData.latitude, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.latitude, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 3:
                if (val_latitude_H(serial1Data.token)) {
                    memset(gnggaData.latitude_hemisphere, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.latitude_hemisphere, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 4:
                if (val_longitude(serial1Data.token)) {
                    memset(gnggaData.longitude, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.longitude, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 5:
                if (val_longitude_H(serial1Data.token)) {
                    memset(gnggaData.longitude_hemisphere, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.longitude_hemisphere, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 6:
                if (val_positioning_status_gngga(serial1Data.token)) {
                    memset(gnggaData.solution_status, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.solution_status, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 7:
                if (val_satellite_count(serial1Data.token)) {
                    memset(gnggaData.satellite_count, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.satellite_count, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 8:
                if (val_gps_precision_factor(serial1Data.token)) {
                    memset(gnggaData.gps_precision_factor, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.gps_precision_factor, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 9:
                if (val_altitude(serial1Data.token)) {
                    memset(gnggaData.altitude, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.altitude, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 10:
                if (val_altitude_units(serial1Data.token)) {
                    memset(gnggaData.altitude_units, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.altitude_units, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 11:
                if (val_geoidal(serial1Data.token)) {
                    memset(gnggaData.geoidal, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.geoidal, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 12:
                if (val_geoidal_units(serial1Data.token)) {
                    memset(gnggaData.geoidal_units, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.geoidal_units, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 13:
                if (val_differential_delay(serial1Data.token)) {
                    memset(gnggaData.differential_delay, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.differential_delay, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 14:
                serial1Data.token = strtok(serial1Data.token, "*");
                if (val_basestation_id(serial1Data.token)) {
                    memset(gnggaData.base_station_id, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnggaData.base_station_id, serial1Data.token);
                    gnggaData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnggaData.bad_element_count[serial1Data.iter_token]++;
                    gnggaData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 15:
                // Skip checksum
                break;
            default:
                break;
        }
        serial1Data.token = strtok(NULL, ",");
        serial1Data.iter_token++;
    }

    gngga_total_bad_elements = 0;
    for (int i = 0; i < MAX_GNGGA_ELEMENTS; i++) {
        if (gnggaData.bad_element_bool[i]) gngga_total_bad_elements++;
        if (gnggaData.bad_element_count[i] >= gnggaData.max_bad) {
            gnggaData.bad_element_count[i] = 0;
        }
    }
    gnggaData.total_bad_elements = gngga_total_bad_elements;
}

// ----------------------------------------------------------------------------------------
// GNRMC Parsing.
// ----------------------------------------------------------------------------------------
void GNRMC(void) {
    serial1Data.iter_token = 0;
    serial1Data.token = strtok(gnrmcData.sentence, ",");
    while (serial1Data.token != NULL) {
        switch (serial1Data.iter_token) {
            case 0:
                if (strcmp(serial1Data.token, "$GNRMC") == 0) {
                    memset(gnrmcData.tag, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.tag, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 1:
                if (val_utc_time(serial1Data.token)) {
                    memset(gnrmcData.utc_time, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.utc_time, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 2:
                if (val_positioning_status_gnrmc(serial1Data.token)) {
                    memset(gnrmcData.positioning_status, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.positioning_status, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 3:
                if (val_latitude(serial1Data.token)) {
                    memset(gnrmcData.latitude, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.latitude, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 4:
                if (val_latitude_H(serial1Data.token)) {
                    memset(gnrmcData.latitude_hemisphere, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.latitude_hemisphere, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 5:
                if (val_longitude(serial1Data.token)) {
                    memset(gnrmcData.longitude, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.longitude, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 6:
                if (val_longitude_H(serial1Data.token)) {
                    memset(gnrmcData.longitude_hemisphere, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.longitude_hemisphere, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 7:
                if (val_ground_speed(serial1Data.token)) {
                    memset(gnrmcData.ground_speed, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.ground_speed, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 8:
                if (val_ground_heading(serial1Data.token)) {
                    memset(gnrmcData.ground_heading, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.ground_heading, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 9:
                if (val_utc_date(serial1Data.token)) {
                    memset(gnrmcData.utc_date, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.utc_date, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 10:
                if (val_installation_angle(serial1Data.token)) {
                    memset(gnrmcData.installation_angle, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.installation_angle, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 11:
                if (val_installation_angle_direction(serial1Data.token)) {
                    memset(gnrmcData.installation_angle_direction, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.installation_angle_direction, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 12:
                serial1Data.token = strtok(serial1Data.token, "*");
                if (val_mode_indication(serial1Data.token)) {
                    memset(gnrmcData.mode_indication, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gnrmcData.mode_indication, serial1Data.token);
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gnrmcData.bad_element_count[serial1Data.iter_token]++;
                    gnrmcData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 13:
                // Skip checksum
                break;
            default:
                break;
        }
        serial1Data.token = strtok(NULL, ",");
        serial1Data.iter_token++;
    }

    gnrmc_total_bad_elements = 0;
    for (int i = 0; i < MAX_GNRMC_ELEMENTS; i++) {
        if (gnrmcData.bad_element_bool[i]) gnrmc_total_bad_elements++;
        if (gnrmcData.bad_element_count[i] >= gnrmcData.max_bad) {
            gnrmcData.bad_element_count[i] = 0;
        }
    }
    gnrmcData.total_bad_elements = gnrmc_total_bad_elements;
}

// ----------------------------------------------------------------------------------------
// GPATT Parsing.
// ----------------------------------------------------------------------------------------
void GPATT(void) {
    serial1Data.iter_token = 0;
    serial1Data.token = strtok(gpattData.sentence, ",");
    while (serial1Data.token != NULL) {
        switch (serial1Data.iter_token) {
            case 0:
                if (strcmp(serial1Data.token, "$GPATT") == 0) {
                    memset(gpattData.tag, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.tag, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 1:
                if (val_pitch_gpatt(serial1Data.token)) {
                    memset(gpattData.pitch, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.pitch, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 2:
                if (val_angle_channle_p_gpatt(serial1Data.token)) {
                    memset(gpattData.angle_channel_0, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.angle_channel_0, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 3:
                if (val_roll_gpatt(serial1Data.token)) {
                    memset(gpattData.roll, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.roll, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 4:
                if (val_angle_channle_r_gpatt(serial1Data.token)) {
                    memset(gpattData.angle_channel_1, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.angle_channel_1, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 5:
                if (val_yaw_gpatt(serial1Data.token)) {
                    memset(gpattData.yaw, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.yaw, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 6:
                if (val_angle_channle_y_gpatt(serial1Data.token)) {
                    memset(gpattData.angle_channel_2, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.angle_channel_2, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 7:
                if (val_software_version_gpatt(serial1Data.token)) {
                    memset(gpattData.software_version, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.software_version, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 8:
                if (val_version_channel_s_gpatt(serial1Data.token)) {
                    memset(gpattData.version_channel, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.version_channel, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 9:
                if (val_product_id_gpatt(serial1Data.token)) {
                    memset(gpattData.product_id, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.product_id, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 10:
                if (val_id_channel_gpatt(serial1Data.token)) {
                    memset(gpattData.id_channel, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.id_channel, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 11:
                if (val_ins_gpatt(serial1Data.token)) {
                    memset(gpattData.ins, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.ins, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 12:
                if (val_ins_channel_gpatt(serial1Data.token)) {
                    memset(gpattData.ins_channel, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.ins_channel, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 13:
                if (val_hardware_version_gpatt(serial1Data.token)) {
                    memset(gpattData.hardware_version, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.hardware_version, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 14:
                if (val_run_state_flag_gpatt(serial1Data.token)) {
                    memset(gpattData.run_state_flag, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.run_state_flag, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 15:
                if (val_mis_angle_num_gpatt(serial1Data.token)) {
                    memset(gpattData.mis_angle_num, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.mis_angle_num, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 16:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_0, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_0, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 17:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_1, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_1, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 18:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_2, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_2, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 19:
                if (val_static_flag_gpatt(serial1Data.token)) {
                    memset(gpattData.static_flag, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.static_flag, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 20:
                if (val_user_code_gpatt(serial1Data.token)) {
                    memset(gpattData.user_code, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.user_code, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 21:
                if (val_gst_data_gpatt(serial1Data.token)) {
                    memset(gpattData.gst_data, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.gst_data, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 22:
                if (val_line_flag_gpatt(serial1Data.token)) {
                    memset(gpattData.line_flag, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.line_flag, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 23:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_3, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_3, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 24:
                if (val_mis_att_flag_gpatt(serial1Data.token)) {
                    memset(gpattData.mis_att_flag, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.mis_att_flag, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 25:
                if (val_imu_kind_gpatt(serial1Data.token)) {
                    memset(gpattData.imu_kind, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.imu_kind, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 26:
                if (val_ubi_car_kind_gpatt(serial1Data.token)) {
                    memset(gpattData.ubi_car_kind, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.ubi_car_kind, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 27:
                if (val_mileage_gpatt(serial1Data.token)) {
                    memset(gpattData.mileage, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.mileage, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 28:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_4, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_4, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 29:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_5, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_5, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 30:
                if (val_run_inetial_flag_gpatt(serial1Data.token)) {
                    memset(gpattData.run_inetial_flag, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.run_inetial_flag, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 31:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_6, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_6, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 32:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_7, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_7, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 33:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_8, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_8, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 34:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_9, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_9, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 35:
                if (val_speed_enable_gpatt(serial1Data.token)) {
                    memset(gpattData.speed_enable, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.speed_enable, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 36:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_10, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_10, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 37:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.custom_logo_11, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.custom_logo_11, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 38:
                if (val_speed_num_gpatt(serial1Data.token)) {
                    memset(gpattData.speed_num, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.speed_num, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 39:
                if (val_custom_flag(serial1Data.token)) {
                    memset(gpattData.scalable, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.scalable, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            case 40:
                serial1Data.token = strtok(serial1Data.token, "*");
                if (val_checksum(serial1Data.token)) {
                    memset(gpattData.check_sum, 0, MAX_GLOBAL_ELEMENT_SIZE);
                    strcpy(gpattData.check_sum, serial1Data.token);
                    gpattData.bad_element_bool[serial1Data.iter_token] = false;
                } else {
                    gpattData.bad_element_count[serial1Data.iter_token]++;
                    gpattData.bad_element_bool[serial1Data.iter_token] = true;
                }
                break;
            default:
                break;
        }
        serial1Data.token = strtok(NULL, ",");
        serial1Data.iter_token++;
    }

    gpatt_total_bad_elements = 0;
    for (int i = 0; i < MAX_GPATT_ELEMENTS; i++) {
        if (gpattData.bad_element_bool[i]) gpatt_total_bad_elements++;
        if (gpattData.bad_element_count[i] >= gpattData.max_bad) {
            gpattData.bad_element_count[i] = 0;
        }
    }
    gpattData.total_bad_elements = gpatt_total_bad_elements;
}

// ------------------------------------------------------------------------------------------------------------------------------
// readGPS
// ------------------------------------------------------------------------------------------------------------------------------
void readGPS(void) {
    serial1Data.gngga_bool=false;
    serial1Data.gnrmc_bool=false;
    serial1Data.gpatt_bool=false;
    while (1) {
      // --------------------------------------------------------------------------
      // Read sentences coming from the WTGPS300P. (WTGPS300P outputs every 100ms).
      // --------------------------------------------------------------------------
      if (Serial1.available()) {
        // Serial.println("[gps data found]");
        // ----------------------------------------------------------------------
        // Break immediately if everything of interest has been collected.
        // ----------------------------------------------------------------------
        if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
        // ----------------------------------------------------------------------
        // Read serial into buffer until buffer max or special char.
        // ----------------------------------------------------------------------
        memset(serial1Data.BUFFER, 0, sizeof(serial1Data.BUFFER));
        serial1Data.nbytes=Serial1.readBytesUntil('\n', serial1Data.BUFFER, sizeof(serial1Data.BUFFER));
        // Serial.println(serial1Data.BUFFER);
        // ----------------------------------------------------------------------
        // Exclude partial reads.
        // ----------------------------------------------------------------------
        if (serial1Data.nbytes>10) {
          // -------------------------------------------------
          // Read GNGGA.
          // -------------------------------------------------   
          if (serial1Data.gngga_bool==false) {
            if (strncmp(serial1Data.BUFFER, "$GNGGA", 6)==0) {
                memset(gnggaData.sentence, 0, sizeof(gnggaData.sentence));
                strcpy(gnggaData.sentence, serial1Data.BUFFER);
                serial1Data.gngga_bool=true;
                if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
            }
          }
          // -------------------------------------------------
          // Read GNRMC.
          // -------------------------------------------------   
          else if (serial1Data.gnrmc_bool==false) {
            if (strncmp(serial1Data.BUFFER, "$GNRMC", 6)==0) {
                memset(gnrmcData.sentence, 0, sizeof(gnrmcData.sentence));
                strcpy(gnrmcData.sentence, serial1Data.BUFFER);
                serial1Data.gnrmc_bool=true;
                if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
            }
          }
          // -------------------------------------------------
          // Read GPATT.
          // -------------------------------------------------   
          else if (serial1Data.gpatt_bool==false) {
            if (strncmp(serial1Data.BUFFER, "$GPATT", 6)==0) {
                memset(gpattData.sentence, 0, sizeof(gpattData.sentence));
                strcpy(gpattData.sentence, serial1Data.BUFFER);
                serial1Data.gpatt_bool=true;
                if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
            }
          }
        }
      }
      delay(1);
    }
}

// ------------------------------------------------------------------------------------------------------------------------------
// Isolate the checksum.
// ------------------------------------------------------------------------------------------------------------------------------
int getCheckSumSerial1(char * string) {
  for (serial1Data.XOR=0, serial1Data.i_XOR=0; serial1Data.i_XOR < strlen(string); serial1Data.i_XOR++) {
    serial1Data.c_XOR=(unsigned char)string[serial1Data.i_XOR];
    if (serial1Data.c_XOR=='*') break;
    if (serial1Data.c_XOR != '$') serial1Data.XOR ^= serial1Data.c_XOR;
  }
  return serial1Data.XOR;
}

// ------------------------------------------------------------------------------------------------------------------------------
// Checksum validator.
// ------------------------------------------------------------------------------------------------------------------------------
bool validateChecksumSerial1(char * buffer) {
  memset(serial1Data.gotSum, 0, sizeof(serial1Data.gotSum));
  serial1Data.gotSum[0]=buffer[strlen(buffer) - 3];
  serial1Data.gotSum[1]=buffer[strlen(buffer) - 2];
  serial1Data.checksum_of_buffer= getCheckSumSerial1(buffer);
  serial1Data.checksum_in_buffer=h2d2(serial1Data.gotSum[0], serial1Data.gotSum[1]);
  if (serial1Data.checksum_in_buffer==serial1Data.checksum_of_buffer) {return true;}
  return false;
}

// ------------------------------------------------------------------------------------------------------------------------------
// Data validate.
// ------------------------------------------------------------------------------------------------------------------------------
void validateGPSData(void) {
    // --------------------------------------------------------------------------
    // Parse data if all sentences have been collected.
    // --------------------------------------------------------------------------
    if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {
      // -------------------------------------------------
      // Check GNGGA.
      // -------------------------------------------------
      memset(gnggaData.outsentence, 0, sizeof(gnggaData.outsentence));
      strcpy(gnggaData.outsentence, gnggaData.sentence);
      gnggaData.valid_checksum=validateChecksumSerial1(gnggaData.sentence);
      if (gnggaData.valid_checksum==true) {GNGGA();}
      // -------------------------------------------------
      // Read GNRMC.
      // -------------------------------------------------
      memset(gnrmcData.outsentence, 0, sizeof(gnrmcData.outsentence));
      strcpy(gnrmcData.outsentence, gnrmcData.sentence);
      gnrmcData.valid_checksum=validateChecksumSerial1(gnrmcData.sentence);
      if (gnrmcData.valid_checksum==true) {GNRMC();}
      // -------------------------------------------------
      // Read GPATT.
      // -------------------------------------------------
      memset(gpattData.outsentence, 0, sizeof(gpattData.outsentence));
      strcpy(gpattData.outsentence, gpattData.sentence);
      gpattData.valid_checksum=validateChecksumSerial1(gpattData.sentence);
      if (gpattData.valid_checksum==true) {GPATT();}
    }
}