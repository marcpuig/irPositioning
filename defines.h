#pragma once

/***********************************
 ** DO NOT CHANGE THESE CONSTANTS **
 ***********************************/
#define PI 3.1415926535897932384626433832795f
#define RAD2DEG 57.295779513082320876798154814105f  // 180 / PI
#define FOV_ROLL 0.933751149817f  // 53.5 * PI / 180.
#define FOV_PITCH 0.722740843251f  // 41.41 * PI / 180.
#define FOV_ROLL_D 53.5
#define FOV_PITCH_D 41.41
#define MAX_PIXELS_X 2592
#define MAX_PIXELS_Y 1944
#define FOCAL_LENGTH 0.00356096050f
#define RED_LED 6
#define GREEN_LED 13
#define QUIET 0
#define ERROR 1
#define WARNING 2
#define NOTICE 3
#define DEBUG 4

/*******************
 ** CONFIGURATION **
 *******************/
#define FPS 40
#define PIXELS_X 1280 // 1920 // 1280 // 1280 // 2592 // 1920 //1280 // 640
#define PIXELS_Y 960  // 1440 // 960  // 720  // 1944 // 1080 // 720 // 480
#define D_PER_PIXEL 0.0000014 * (MAX_PIXELS_X / PIXELS_X)
#define MIN_BEACON_PIXELS 1
#define MAX_BEACON_PIXELS 28000
#define MAX_BEACON_DIFF_FACTOR 3
#define DISTANCE_FACTOR 3.0
#define DISTANCE_MARGIN 0.00275
#define ANGLE_MARGIN 20.0
#define THRESHOLD 80
#define MAX_POSSIBLE_BEACONS 100
#define MAX_BEACONS 24
#define MAX_WAYPOINTS 4
#define MAX_WAYPOINT_BEACONS 5
#define LOW_PASS_POS_FILTER 0.00
#define LOW_PASS_VEL_FILTER 0.5
#define DISTANCE_TO_RELOC 0.02
#define ANGLE_OFFSET_TO_RELOC 1
#define YAW_OFFSET_DIVIDER 10
#define HIST_SIZE 10
#define SWITCH_WAYPOINT_MARGIN 0.075
#define ATTITUDE_DELAY 37000
//#define THUMBNAIL

// Camera servos
#define ROLL_SERVO_SPEED 220 // 500  // deg/s
#define PITCH_SERVO_SPEED 220 // 500 // deg/s
#define ROLL_SERVO_DELAY 60000 // us
#define PITCH_SERVO_DELAY 60000 // us
#define PWM_MID 1500
#define ROLL_PWM_RANGE 500
#define PITCH_PWM_RANGE 500
#define ROLL_RANGE 53.00
#define PITCH_RANGE 53.00
#define ROLL_OFFSET -7.7f
#define PITCH_OFFSET 2.3f
#define HOME_SERVOS_ON_LOST_DELAY 2500000

// Check for bad waypoints
#define MAX_WAYPOINT_DISTANCE_MULTIPLIER 4
#define MAX_WAYPOINT_HEADING_CHANGE 15
#define MAX_WAYPOINT_HEIGHT_CHANGE 0.1
#define MAX_WAYPOINT_BORDER_DISTANCE_MULTIPLIER 2

// Debug
#define DEBUGL NOTICE

#define DFB 0x01  // Find Beacons
#define DGB 0x02  // Group Beacons
#define DFW 0x04  // Find Waypoints
#define DCP 0x08  // Compute Position

#define DEBUG_CONF DFB //(DFB | DGB | DFW | DCP)

#define DEBUG_SPREADSHEET 0

#if (DEBUGL > O && !DEBUG_SPREADSHEET)
  #define TRACE(message_level, format, ...) if(message_level <= DEBUGL) printf(format, ##__VA_ARGS__)
  #define TRACET(message_level, type, format, ...) if(message_level <= DEBUGL && type & DEBUG_CONF) printf(format, ##__VA_ARGS__)
  #define STARTCHRONO Chrono c = Chrono();
  #define GETCLICK c.click()
#else
  #define TRACE(format, ...)
  #define TRACET(format, ...)
  #define STARTCHRONO
  #define GETCLICK
#endif

#if DEBUG_SPREADSHEET
  #define TRACETS(message_level, type, format, ...) if(message_level <= DEBUGL && type & DEBUG_CONF) printf(format, ##__VA_ARGS__)
#else
  #define TRACETS(format, ...)
#endif
