#include "mavlink.h"

struct STREAM_DATA {
  uint16_t battVoltage = 0;
  float altitude = 0;
  float climb = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  uint8_t gps_sats = 0;
  float gps_lon = 0;
  float gps_lat = 0;
  float home_lon = 0;
  float home_lat = 0;
  uint16_t gps_speed = 0;
  boolean gps_fix = false;
  uint32_t home_distance = 0;
};

STREAM_DATA streamData;

boolean set = false;


void setup() {
  Serial.begin(115200);
  Serial1.begin(57600);
  delay(1000);
  Serial.println("Starting up...");

}

void loop() {
  comm_receive();
}

void comm_receive() {

  mavlink_message_t msg;
  mavlink_status_t status;
  //receive data over serial
  while (Serial1.available() > 0) {

    if (mavlink_parse_char(MAVLINK_COMM_0, Serial1.read(), &msg, &status)) {
      // Handle message
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {


            break;
          }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            streamData.gps_sats = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
            streamData.gps_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0000f;
            streamData.gps_lat = mavlink_msg_gps_raw_int_get_lat(&msg)  / 10000000.0000f;
            streamData.gps_speed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
            
            
            
//                        Serial.print("gps_sats: ");
//                        Serial.println(streamData.gps_sats);
//                        Serial.print("gps_lon: ");
//                        Serial.println(streamData.gps_lon);
//                        Serial.print("gps_latsats: ");
//                        Serial.println(streamData.gps_lat);
//                        Serial.print("gps_speed: ");
//                        Serial.println(streamData.gps_speed);
            
            if (!streamData.gps_fix) {
              if (mavlink_msg_gps_raw_int_get_fix_type(&msg) >= 3) {

                Serial.print("Setting home location: ");
                Serial.print(streamData.gps_lon);
                Serial.print("/");
                Serial.println(streamData.gps_lat);
                streamData.gps_fix = true;
                streamData.home_lon =  streamData.gps_lon;
                streamData.home_lat =  streamData.gps_lat;
              }
            }


            break;
          }

        case MAVLINK_MSG_ID_VFR_HUD: {
            streamData.altitude = mavlink_msg_vfr_hud_get_alt(&msg);
            streamData.climb = mavlink_msg_vfr_hud_get_climb(&msg);
            break;
          }

        case MAVLINK_MSG_ID_ATTITUDE: {
            streamData.roll =  mavlink_msg_attitude_get_roll(&msg);
            streamData.pitch =  mavlink_msg_attitude_get_pitch(&msg);
            streamData.yaw =  mavlink_msg_attitude_get_yaw(&msg);
            float deg = streamData.yaw * 180 / PI;
            if(deg < 0)
            {
              deg = 180 - abs(deg);
              deg = 180 + deg;
            }
            Serial.println(deg);
            break;
          }

        case MAVLINK_MSG_ID_SYS_STATUS: {
            streamData.battVoltage = mavlink_msg_sys_status_get_voltage_battery(&msg);
            break;
          }
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
            streamData.gps_fix = true;
            streamData.home_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
            break;
          }
      }

    }
    // And get the next one
  }
}
#define d2r (M_PI / 180.0)
float calc_dist(float lat1, float long1, float lat2, float long2) {
  double dlong = (long2 - long1) * d2r;
  double dlat = (lat2 - lat1) * d2r;
  double a = pow(sin(dlat / 2.0), 2) + cos(lat1 * d2r) * cos(lat2 * d2r) * pow(sin(dlong / 2.0), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = 6367 * c * 1000; // in meters
  if (d >= 0) {
    return d;
  } else {
    return -d;
  }
}

void set1() {

  int  alt = streamData.altitude * 10;
  byte  altHi = highByte(alt )  ;
  byte  altLo =  lowByte(alt ) ;

  int  yaw = streamData.yaw * 100;
  byte yawHi = highByte(yaw );
  byte yawLo = lowByte(yaw );


  int  speed2 = streamData.gps_speed * 10;
  byte speedHi = highByte(speed2);
  byte speedLo = lowByte(speed2);


  int  roll = streamData.roll * 10;
  byte rollHi = highByte(roll);
  byte rollLo = lowByte(roll);


  int  pitch = streamData.pitch * 10;
  byte pitchHi = highByte(pitch);
  byte pitchLo = lowByte(pitch);

  int distance = calc_dist(streamData.gps_lat, streamData.gps_lon, streamData.home_lat, streamData.home_lon) * 100;
  byte distanceHi = highByte(distance);
  byte distanceLo = lowByte(distance);

  byte buffer[16] = {0x89, 0xAB, streamData.gps_sats, altHi, altLo, yawHi, yawLo, speedHi, speedLo, rollHi , rollLo, pitchHi, pitchLo, distanceHi, distanceLo, 0x00};
  Serial.write(buffer, 16);
}
void set2() {

  int  rise = streamData.climb * 10;
  byte  riseHi = highByte(rise);
  byte  riseLo =  lowByte(rise);

  byte voltesHi = highByte(streamData.battVoltage);
  byte voltesLo = lowByte(streamData.battVoltage);

  float lat = 40.689060;
  float log = -74.044636;

  byte buffer[16] = {0x89, 0xCD, streamData.gps_sats, riseHi, riseLo, voltesHi, voltesLo,/**/ 0x0, 0x0, 0x0 , 0x0, /**/0x0, 0x0, 0x0, 0x0,/**/ 0x00};
  Serial.write(buffer, 16);
}


void onRequest() {
  if (set) {
    set = false;
    set1();
  } else {
    set = true;
    set2();
  }
}
