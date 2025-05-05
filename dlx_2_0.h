/*
 * dlx_2_0.h
 *
 *  Created on: Apr 17, 2025
 *      Author: jhfjf
 */

#ifndef INC_DXL_2_0_H_
#define INC_DXL_2_0_H_
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
	DXL_1_Goal_Torque_Limit = 34,
	DXL_1_Goal_Position = 30,
	DXL_1_Present_Load = 40,
	DXL_1_Present_Position = 36,
	DXL_1_Present_Temperature = 43,
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
    uint8_t length;        // 1바이트 길이
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



#endif /* INC_DXL_2_0_H_ */
