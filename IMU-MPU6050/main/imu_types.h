#ifndef IMU_TYPES_H_
#define IMU_TYPES_H_

 typedef union {
   struct {
         int16_t x;
         int16_t y;
         int16_t z;
   };
   int16_t axis[3];
 } Axis3i16;

 typedef union {
   struct {
         int32_t x;
         int32_t y;
         int32_t z;
   };
   int32_t axis[3];
 } Axis3i32;

 typedef union {
   struct {
         int64_t x;
         int64_t y;
         int64_t z;
   };
   int64_t axis[3];
 } Axis3i64;

 typedef union {
   struct {
         float x;
         float y;
         float z;
   };
   float axis[3];
 } Axis3f;

 typedef struct {
	 float x;
	 float y;
	 float z;
 } AXIS;

 typedef struct _pidpara{
	 float roll[3];
	 float pitch[3];
	 float yaw[2];
	 float k;
 } PidParam;

 typedef enum {
	 KP = 0,
	 KI,
	 KD
 } PID_ID;

 typedef struct _gps_data {
	 float latitude;
	 float longitude;
	 float altitude;
 } GPS_Data;

 typedef struct _info {
 	AXIS attitude;
 	GPS_Data gps;
 	float thrust[4];
 	float bat;
 	uint8_t status;
 }Info;

#endif /* IMU_TYPES_H_ */
