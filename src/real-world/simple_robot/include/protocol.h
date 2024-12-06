#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#define HEADER_SOF 0xA5
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#pragma pack(push, 1)
typedef enum
{
  CHASSIS_ODOM_CMD_ID = 0x0301,
  CHASSIS_CTRL_CMD_ID = 0x0302,
  CHASSIS_PUMP_CMD_ID = 0X0303,
} referee_data_cmd_id_type;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef struct
{

  float S1_angle;
  float S2_angle;
  float S3_angle;
  float S4_angle;

}  chassis_odom_info_t;


typedef struct
{

  float wheel_speed[4];
  float wheel_angle[4];
  uint8_t brake;

}  chassis_ctrl_info_t;

typedef struct
{

 uint8_t pump1_en;
 uint8_t pump2_en;

} pump_ctrl_info_t;



#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H
