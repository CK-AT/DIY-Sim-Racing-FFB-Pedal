#ifndef MODBUS_H
#define MODBUS_H

//   #ifdef MODEBUS_LOG
//  #define MODEBUS_LOG


#include <Arduino.h>
#include <Stream.h>
#include <vector>
using namespace std;

class Modbus
{
private:
    /* data */
    bool log = false;
    int mode_ = -1;
    uint32_t timeout_ = 20;
    HardwareSerial* s ;
    uint8_t rawRx[512];
    int  lenRx = 0;
    int  datalen = 0;
    int  SlaveID = 0x01;
    uint8_t txout[14] = {};
    #define Coil_Register       0x01
    #define Discret_Register    0x02
    #define Holding_Register    0x03
    #define Input_Register      0x04  
	#define Write_Holding_Register      0x06
	#define Write_Multiple_Registers      0x10
    // vector <char> txbuff;
    // vector <char> rxbuff;
    
public:
    
    Modbus();
    Modbus(HardwareSerial &st);
    
    bool init(int mode, bool en_log = false);
    void setTimeout(uint16_t timeout);


    uint8_t byteRead(int nb);
    template <typename T>
    T blockRead(int index)
    {
        return  T((rawRx[index*2+3] << 8) | rawRx[index*2+4]);
    }
    template <typename T>
    int32_t holdingRegisterRead(int id, int address, T &value)
    {
        int block = constrain(sizeof(T) / 2, 1, 2);
        if(requestFrom(SlaveID, Holding_Register, address, block) > 0)
        {
            if(block == 2)
            {
            value = T(blockRead<uint16_t>(0) << 16 | blockRead<uint16_t>(1));
            }
            else{
                value = blockRead<T>(0);
            }
            return 0;
        }
        else{
            return -1;
        }
    }
    template <typename T>
    T inputRegisterRead(int id, int address, int block)
    {
        if(block > 2){block = 2;}
        if(requestFrom(id, Input_Register,address,block))
        {
            if(block == 2)
            {
            return T(blockRead<uint16_t>(0) << 16 | blockRead<uint16_t>(1));
            }
            else{
            return blockRead<T>(0);
            }
        }
        else
        {
            return -1;
        }
    }

    int coilRead(int address);                                      //Return 1 byte = 8 bit coil
    int coilRead(int id, int address);
    int discreteInputRead(int address);
    int discreteInputRead(int id, int address);
    long holdingRegisterRead(int address);
    long holdingRegisterRead(int id, int address, int block);
    long inputRegisterRead(int address);
    long inputRegisterRead(int id, int address, int block);
    
    int coilWrite(int address, uint8_t value);
    int coilWrite(int id, int address, uint8_t value);
    int holdingRegisterWrite(int address, uint16_t value);
    int holdingRegisterWrite(int id, int address, uint16_t value);
    int holdingRegisterWriteI32(int id, int address, int32_t value);
    void RxRaw(uint8_t *raw, uint8_t &rlen);
    void TxRaw(uint8_t *raw, uint8_t &rlen);
    //Read multiple coils, discrete inputs, holding registers, or input register values.
    //int requestFrom(int type, int address, int nb, byte *ret,int len);
    int requestFrom(int slaveId, int type, int address,int nb);
    //  ~Modbus();
    bool checkAndReplaceParameter(uint16_t slaveId_local_u16, uint16_t parameterAdress, long value);
    int16_t readParameter(uint16_t slaveId_local_u16, uint16_t parameterAdress);


    // Read Coil Register       0x01
    int ReadCoilReg(int add);
    int ReadCoilReg(int slaveId, int add);
    int ReadCoilReg(int slaveId, int add, int nbit);

    // Read Discret Register    0x02
    int ReadDiscretReg(int add);
    int ReadDiscretReg(int slaveId, int add);
    int ReadDiscretReg(int slaveId, int add, int nbit);

    // Read Holding Register    0x03
    int ReadHoldingReg(int add); 
    int ReadHoldingReg(int slaveId, int add);
    int ReadHoldingReg(int slaveId, int add, int nbyte);

    // Read Input Register      0x04
    int ReadInputReg(int add);
    int ReadInputReg(int slaveId, int add);
    int ReadInputReg(int slaveId, int add, int nbyte);


    int8_t   uint8(int add);
    uint16_t uint16(int add);
    uint32_t uint32(int add, bool byteHL = true);
    


    int CheckCRC(uint8_t *buf, int len);
};

//  #else
//   #error "Log not defined"
//  #endif

#endif
