// ################################################//
// Test Program to initialise and grab data from   //
// Murata 7 axis orientation sensor module to an   //
// Arduino UNO.                                    //
// Author: Killian O'Sullivan                      //
// Majentix Ltd (2017)                             //
// Assigned to: Rod Kashani                        //
// V1                                              //
// ################################################// 
#include <SPI.h>

#define VERSION_NUM 1

// PB2 pin 10 on the UNO header,
// chip select to SCC2230 pin 13 (CSB) on MuRata header
#define nCSB 10

// PD7 pin 7 on the UNO,
// chip select to SCC2130 pin 9 (EXTRA1) on MuRata header
#define nCS1 7

// PB0 pin 8 on the UNO header,
// chip select to SCC2100  pin 8 (EXTRA2) on MuRata header
#define nCS2 8

// PB1 Pin 9 on UNO header,
// hard external reset to all 3 devices.
#define nRESET 9

#define MOSI 11
#define MISO 12
#define SCLK 13
#define SPI_CLK_RATE  4000000
#define MODE_ALL_CLEAR_0  0b00100101  //0x08 no shifting, sent as is
#define SPI_BITS_PER_WORD  8  // number of bits (Only 8 works)
#define BAUD_RATE 9600

// murata sensor SPI commands
// Standard Requests
const unsigned long REQ_READ_RATE = 0x040000f7;
const unsigned long REQ_READ_ACC_X = 0x100000e9;
const unsigned long REQ_READ_ACC_Y = 0x140000ef;
const unsigned long REQ_READ_ACC_Z = 0x180000e5;
const unsigned long REQ_READ_TEMP = 0x1c0000e3;
const unsigned long REQ_WRITE_FLT_60 =  0xfc200006;
const unsigned long REQ_WRITE_FLT_10 = 0xfc1000c7;
const unsigned long REQ_READ_STAT_SUM = 0x7c0000b3;
const unsigned long REQ_READ_RATE_STAT1 = 0x240000c7;
const unsigned long REQ_READ_RATE_STAT2 = 0x280000cd;
const unsigned long REQ_READ_ACC_STAT = 0x3c00003d;
const unsigned long REQ_READ_COM_STAT1 = 0x6c0000ab;
// Special requests
const unsigned long REQ_HARD_RESET = 0xD8000431;


// Function prototypes
unsigned long initialise_device(int);
unsigned long spi_transfer(unsigned long, int);
void setup(void);
void reset_sensor_module(void);
int  check_status_ok(unsigned long);
void test_status(void);
byte calc_crc(unsigned long);
static byte CRC8(unsigned long, byte);

void loop()
{
    unsigned long response_stat_sum;
    unsigned long rate;
    unsigned long acc_x;
    unsigned long acc_y;
    unsigned long acc_z;
    const char *ident[3];
    int chip_select_list[3],chip_select, i;
    char srate[12], sacc_x[12], sacc_y[12], sacc_z[12];

    chip_select_list[0] = nCSB;
    chip_select_list[1] = nCS1;
    chip_select_list[2] = nCS2;
    ident[0] = "nCSB";
    ident[1] = "nCS1";
    ident[2] = "nCS2";

    // All read commands are 1 frame delayed so when a write is performed, the read data is
    // from a register of the previous write.
    for (chip_select = 0; chip_select < 3; chip_select++) {
        response_stat_sum = spi_transfer(REQ_READ_RATE, chip_select);
    }

    Serial.print("ID, rate, a_x, a_y, a_z\n");

    for(i = 0; i < 8; i++) {
        chip_select = 0;
        rate =  spi_transfer(REQ_READ_ACC_X, chip_select_list[chip_select]);
        acc_x = spi_transfer(REQ_READ_ACC_Y, chip_select_list[chip_select]);
        acc_y = spi_transfer(REQ_READ_ACC_Z, chip_select_list[chip_select]);    
        acc_z = spi_transfer(REQ_READ_RATE, chip_select_list[chip_select]);

        // debug section //
        sprintf(srate, "%d", ( rate >> 8UL) & 0xFFFF);
        sprintf(sacc_x, "%d", (acc_x >> 8UL) & 0xFFFF);
        sprintf(sacc_y, "%d", (acc_y >> 8UL) & 0xFFFF);
        sprintf(sacc_z, "%d", (acc_z >> 8UL) & 0xFFFF);

        Serial.print(ident[chip_select]);
        Serial.print(", ");
        Serial.print(srate);
        Serial.print(", ");
        Serial.print(sacc_x);
        Serial.print(", ");
        Serial.print(sacc_y);
        Serial.print(", ");
        Serial.println(sacc_z);

    }
    delay(200);
}

//***************************************//
// Functions
//***************************************//
void setup(void)
{
    unsigned long response_stat_sum;
    const char *ident[3];
    int chip_select_list[3],chip_select, i;

    // Setup SPI
    pinMode(nCSB, OUTPUT);
    pinMode(nCS1, OUTPUT);
    pinMode(nCS2, OUTPUT);
    pinMode(nRESET, OUTPUT);
    digitalWrite(nCSB, HIGH);
    digitalWrite(nCS1, HIGH);
    digitalWrite(nCS2, HIGH);
    digitalWrite(nRESET, HIGH);

    // Configure the SPI Library and will define the standard chip SPI pins
    SPI.begin();
    SPI.beginTransaction(SPISettings(SPI_CLK_RATE, MSBFIRST, SPI_MODE0));

    Serial.begin(BAUD_RATE); 
    Serial.println("Murata Orientation Sensor Monitor V");
    Serial.println(VERSION_NUM);
    delay(100); 

    reset_sensor_module();

    // Initialise nCSB (SCC2230)
    response_stat_sum = initialise_device(nCSB);
    if (check_status_ok(response_stat_sum) == 0) {
        Serial.println("SCC2230 has failed Initialisation");
    }
    response_stat_sum = initialise_device(nCS1);  

    if (check_status_ok(response_stat_sum) == 0) {
        Serial.println("SCC2130 has failed Initialisation");
    }
    response_stat_sum = initialise_device(nCS2);  

    if (check_status_ok(response_stat_sum) == 0) {
        Serial.println("SCC2100 has failed Initialisation");
    }

    test_status();
      
}

//***************************************//
void reset_sensor_module()
{
    delay(25);
    digitalWrite(nRESET, LOW);
    delay(2);
    digitalWrite(nRESET, HIGH);
    // wait 25ms for reset to complete and SPI to become active
    delay(35);
}

//***************************************//
unsigned long initialise_device (int ncs)
{
    unsigned long response_stat_sum;
 
    spi_transfer(REQ_WRITE_FLT_60, ncs);
    delay(750);

    // Clear Status Registers
    spi_transfer(REQ_READ_RATE_STAT1,ncs);
    spi_transfer(REQ_READ_RATE_STAT2,ncs);
    spi_transfer(REQ_READ_ACC_STAT,ncs);
    spi_transfer(REQ_READ_COM_STAT1,ncs);
    spi_transfer(REQ_READ_STAT_SUM,ncs);

    // read a 2nd time because of out of frame format
    response_stat_sum = spi_transfer(REQ_READ_STAT_SUM,ncs);
    // read a 3rd time to be safe (maybe unnecessary)
    response_stat_sum = spi_transfer(REQ_READ_STAT_SUM,ncs);
    Serial.print("ResponseStatSum = ");
    Serial.println (response_stat_sum, HEX);
    return response_stat_sum;
  }


//***************************************//
unsigned long spi_transfer(unsigned long data, int ncs)
{
    unsigned long buffer;
    int i;

    // assert chip select
    digitalWrite(ncs, LOW);

    // send ms byte of data and read back and place in msbyte of buffer
    buffer = (unsigned long)SPI.transfer((data>>24)) << 24UL; 
    buffer |= (unsigned long)SPI.transfer((data>>16)) << 16UL;
    buffer |= (unsigned long)SPI.transfer((data>>8)) << 8UL;
    buffer |= (unsigned long)SPI.transfer(data);
    digitalWrite(ncs, HIGH);  // deassert chip select

    return buffer;
}

//***************************************//
int check_status_ok(unsigned long resp_stat_sum)
{
    // check that RS bits are set to 0x01 for normal operation
    if (((resp_stat_sum >> 24) & 0x03) != 0x01) {
        return 0;
    }
    else {
        return 1;
   }
}

//***************************************//
int calculate_temperature (unsigned long temp)
{
    int temperature;

    // need to check if the casting is correct
    temperature = (int)temp;
    temperature = 60 + (temperature / 14.7);
    return temperature;
}


//***************************************//
void test_status()
{
    char gyro_status, acc_status;
    unsigned long temp;
    unsigned long response_stat_sum;
    int temperature;

    spi_transfer(REQ_READ_STAT_SUM,nCSB);
    // perfrom this twice to read the out of frame status summary
    response_stat_sum = spi_transfer(REQ_READ_STAT_SUM,nCSB);

    gyro_status = (response_stat_sum >> 8) & 0x01;
    acc_status = (response_stat_sum >> 11) & 0x01;
    temp = spi_transfer(REQ_READ_TEMP, nCSB);
    temp = (spi_transfer(REQ_READ_TEMP, nCSB) >> 8UL) & 0xFFFF;

    temperature = calculate_temperature(temp);
    Serial.print("response_stat_sum =0x");
    Serial.println(response_stat_sum, HEX);

    Serial.print("gyro status = ");
    Serial.println(gyro_status, DEC);
    Serial.print("accel status = ");
    Serial.println(acc_status, DEC);
    Serial.print("Temperature = ");
    Serial.println(temperature, DEC);

    spi_transfer(REQ_READ_RATE_STAT1, nCSB);
    response_stat_sum = spi_transfer(REQ_READ_RATE_STAT1, nCSB);
    Serial.print("RATE Status 1 Register =0x");
    Serial.println(response_stat_sum, HEX);

    spi_transfer(REQ_READ_RATE_STAT2, nCSB);
    response_stat_sum = spi_transfer(REQ_READ_RATE_STAT2, nCSB);
    Serial.print("RATE Status 2 Register =0x");
    Serial.println(response_stat_sum, HEX);

    spi_transfer(REQ_READ_COM_STAT1, nCSB);
    response_stat_sum = spi_transfer(REQ_READ_COM_STAT1, nCSB);
    Serial.print("COMMON Status Register =0x");
    Serial.println(response_stat_sum, HEX);

    spi_transfer(REQ_READ_ACC_STAT, nCSB);
    response_stat_sum = spi_transfer(REQ_READ_ACC_STAT, nCSB);
    Serial.print("ACC Status Register =0x");
    Serial.println(response_stat_sum, HEX);
}


//***************************************//
// Calculate CRC for 24 MSB's of the 32 bit dword
// (8 LSB's are the CRC field and are not included in CRC calculation)
byte calc_crc(unsigned long data)
{
    unsigned long BitMask;
    unsigned long BitValue;
    byte CRC = 0xFF;
    for (BitMask = 0x80000000; BitMask != 0x80; BitMask >>= 1)
    {
        BitValue = data & BitMask;
        CRC = CRC8(BitValue, CRC);
    }
    CRC = (byte)~CRC;
    return CRC;
}


//***************************************//
static byte CRC8(unsigned long BitValue, byte CRC)
{
    byte Temp = (byte)(CRC & 0x80);
    if (BitValue != 0)
    {
        Temp ^= 0x80;
    }
    CRC <<= 1;
    if (Temp > 0)
    {
        CRC ^= 0x1D;
    }
    return CRC;
}

