/*
 * dlx_2_0.h
 *
 *  Created on: Apr 17, 2025
 *      Author: jhfjf
 */

#ifndef INC_DXL_2_0_H_
#define INC_DXL_2_0_H_

#define leg_count 4
#define all_wheel_motor 4
#define all_ankle_motor 4
#define all_hip_knee_motor 8
#define all_ankle_wheel_motor 8

#define one_leg_wheel_motor 1
#define one_leg_ankle_motor 1
#define one_leg_hip_knee_motor 2
#define one_leg_ankle_wheel_motor 2


// 기본 최대 파라미터 크기 (일반 명령용)
#define DXL_STD_PARAMS 64

// Sync 명령용 파라미터 크기
#define DXL_SYNC_PARAMS 64  // 8모터 × (ID + 데이터(약 9바이트)) + 주소/길이

// Bulk 명령용 파라미터 크기
#define DXL_BULK_PARAMS 128  // 8모터 × (ID + 주소 + 길이 + 데이터)


enum DxlInst{
	DXL_INST_PING = 0x01,
	DXL_INST_READ = 0x02,
	DXL_INST_WRITE = 0x03,
	DXL_INST_REG_WRITE = 0x04,
	DXL_INST_ACTION = 0x05,
	DXL_INST_FACTORY_RESET = 0x06,
	DXL_INST_REBOOT = 0x08,
	DXL_INST_CLEAR = 0x10,
	DXL_INST_STATUS = 0x55,
	DXL_INST_SYNC_READ = 0x82,
	DXL_INST_SYNC_WRITE = 0x83,
	DXL_INST_BULK_READ = 0x92,
	DXL_INST_BULK_WRITE = 0x93,
};

enum Dxl_2_0_Addr{
	DXL_2_Torque_Enable = 64, //1
	DXL_2_LED = 65,//1
	DXL_2_Goal_PWM = 100,//2
	DXL_2_Goal_Position = 116,//4
	DXL_2_Present_Current = 126,//2
	DXL_2_Present_Position = 132,//4
	DXL_2_Present_Temperature = 146,//1
};

enum Dxl_1_0_Addr{
	DXL_1_Torque_Enable = 24,//1
	DXL_1_LED = 25,//1
	DXL_1_Goal_Torque_Limit = 34,//1
	DXL_1_Goal_Position = 30,//2
	DXL_1_Present_Load = 40,//2
	DXL_1_Present_Position = 36,//2
	DXL_1_Present_Temperature = 43,//2
};
// 패킷 타입 열거형
enum PacketType {
	PACKET_SINGLE = 1,
	PACKET_SYNC_WIRTE = 2,
	PACKET_SYNC_READ = 3,
	PACKET_BULK_WIRTE = 4,
	PACKET_BULK_READ = 5,
	PACKET_STATUS = 6
};

enum SendLegType {
	ALL_LEG = 0,
	FR_LEG = 1,
	FL_LEG = 2,
	RR_LEG = 3,
	RL_LEG = 4,
	ALL_ANKLE = 40,
	FR_ANKLE = 41,
	FL_ANKLE = 42,
	RR_ANKLE = 43,
	RL_ANKLE = 44,
	ALL_WHEEL = 50,
	FR_WHEEL = 51,
	FL_WHEEL = 52,
	RR_WHEEL = 53,
	RL_WHEEL = 54,
};


//-------------------single------------------------//
typedef struct {
	uint8_t header[3];     // {0xFF, 0xFF, 0xFD}로 초기화 필요
	uint8_t reserved;      // 0x00으로 초기화 필요
	uint8_t id;
	uint16_t length;       // 2바이트 길이 (Little Endian)
	uint8_t inst;
	uint8_t params[DXL_STD_PARAMS];
	uint16_t crc;          // 2바이트 CRC (Little Endian)
} dxl_packet_2_0;

// 프로토콜 1.0 Instruction 패킷
typedef struct {
	uint8_t header[2];     // {0xFF, 0xFF}로 초기화 필요
	uint8_t id;
	uint8_t length;        // 1바이트 길이
	uint8_t inst;
	uint8_t params[DXL_STD_PARAMS];
	uint8_t checksum;      // 1바이트 Checksum
} dxl_packet_1_0;
//-------------------single------------------------//



//--------------------Sync-------------------------//
// Sync Write 패킷 구조체
typedef struct {
	uint8_t header[2];
	uint8_t id;  // 항상 0xFE (브로드캐스트)
	uint8_t length;
	uint8_t inst;  // 항상 0x83 (Sync Write)
	uint8_t start_addr;
	uint8_t data_len;
	uint8_t params[DXL_SYNC_PARAMS];
	uint8_t checksum;
} dxl_sync_write_1_0;

typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;  // 항상 0xFE (브로드캐스트)
	uint16_t length;
	uint8_t inst;  // 항상 0x83 (Sync Write)
	uint16_t start_addr;
	uint16_t data_len;
	uint8_t params[DXL_SYNC_PARAMS];
	uint16_t crc;
} dxl_sync_write_2_0;

// Sync Read 패킷 구조체 (프로토콜 2.0만 가능)
typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;  // 항상 0xFE (브로드캐스트)
	uint16_t length;
	uint8_t inst;  // 항상 0x82 (Sync Read)
	uint16_t start_addr;
	uint16_t data_len;
	uint8_t params[DXL_SYNC_PARAMS];  // ID 목록
	uint16_t crc;
} dxl_sync_read_2_0;
//--------------------Sync-------------------------//



//--------------------Bulk-------------------------//
// Bulk Read 패킷 구조체
typedef struct {
	uint8_t header[2];
	uint8_t id;  // 항상 0xFE (브로드캐스트)
	uint8_t length;
	uint8_t inst;  // 항상 0x92 (Bulk Read)
	uint8_t params[DXL_BULK_PARAMS];  // [ID1, 주소1, 길이1, ID2, ...]
	uint8_t checksum;
} dxl_bulk_read_1_0;

typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;  // 항상 0xFE (브로드캐스트)
	uint16_t length;
	uint8_t inst;  // 항상 0x93 (Bulk Write)
	uint8_t params[DXL_BULK_PARAMS];  // [ID1, 주소1(2바이트), 길이1(2바이트), 데이터1, ID2, ...]
	uint16_t crc;
} dxl_bulk_write_2_0;

typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;  // 항상 0xFE (브로드캐스트)
	uint16_t length;
	uint8_t inst;  // 항상 0x92 (Bulk Read)
	uint8_t params[DXL_BULK_PARAMS];  // [ID1, 주소1(2바이트), 길이1(2바이트), ID2, ...]
	uint16_t crc;
} dxl_bulk_read_2_0;
//--------------------Bulk-------------------------//



//-------------------Status------------------------//
// 프로토콜 2.0 Status 패킷
typedef struct {
	uint8_t header[3];     // {0xFF, 0xFF, 0xFD}로 초기화 필요
	uint8_t reserved;      // 0x00으로 초기화 필요
	uint8_t id;
	uint16_t length;       // 2바이트 길이 (Little Endian)
	uint8_t inst;          // 일반적으로 0x55 (Status Instruction)
	uint8_t error;
	uint8_t params[DXL_BULK_PARAMS];
	uint16_t crc;          // 2바이트 CRC (Little Endian)
} dxl_status_2_0;

// 프로토콜 1.0 Status 패킷
typedef struct {
	uint8_t header[2];     // {0xFF, 0xFF}로 초기화 필요
	uint8_t id;
	uint8_t length;        // 2바이트 길이
	uint8_t error;
	uint8_t params[DXL_BULK_PARAMS];
	uint8_t checksum;      // 1바이트 Checksum
} dxl_status_1_0;
//-------------------Status------------------------//



//------------------LegMotors------------------------//
typedef struct {
	uint8_t hip;    // MX-106
	uint8_t knee;   // MX-64
	uint8_t ankle;  // AX-12A
	uint8_t wheel;  // AX-12A
} LegMotors;
//------------------LegMotors------------------------//


void send_hello(void);
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length);
dxl_sync_write_1_0 creat_sync_packet_1_0(void);
dxl_sync_write_2_0 creat_sync_packet_2_0(void);
dxl_bulk_read_1_0 creat_bulk_packet_1_0(void);
dxl_bulk_read_2_0 creat_bulk_packet_2_0(void);
uint16_t serialize_sync_write_2_0(dxl_sync_write_2_0 *packet, uint8_t *id_array, uint8_t id_count, uint32_t *data_array, uint8_t *buffer, uint16_t buffer_size);
void sync_write_1(uint8_t send_leg_type, uint16_t addr, uint16_t ankle_data, uint16_t wheel_data);
void send_sync_write_2(uint8_t send_leg_type, uint16_t addr, uint16_t data_len, uint32_t *hip_data, uint32_t *knee_data);

#endif /* INC_DXL_2_0_H_ */
