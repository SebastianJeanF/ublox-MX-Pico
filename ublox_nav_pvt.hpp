#ifndef UBLOX_NAV_PVT_HPP
#define UBLOX_NAV_PVT_HPP

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>

#pragma pack(push, 1) // Ensure there is no byte padding between data members in memory
struct UbxNavPvt {
    uint32_t iTOW;        
    uint16_t year;        
    uint8_t  month;       
    uint8_t  day;         
    uint8_t  hour;        
    uint8_t  min;         
    uint8_t  sec;         
    uint8_t  valid;       
    uint32_t tAcc;        
    int32_t  nano;        
    uint8_t  fixType;     
    uint8_t  flags;       
    uint8_t  flags2;      
    uint8_t  numSV;       
    int32_t  lon;         
    int32_t  lat;         
    int32_t  height;      
    int32_t  hMSL;        
    uint32_t hAcc;        
    uint32_t vAcc;        
    int32_t  velN;        
    int32_t  velE;        
    int32_t  velD;        
    int32_t  gSpeed;      
    int32_t  headMot;     
    uint32_t sAcc;        
    uint32_t headAcc;     
    uint16_t pDOP;        
    uint16_t flags3;      
    uint8_t  reserved0[4];
    int32_t  headVeh;     
    int16_t  magDec;      
    uint16_t magAcc;      
};
#pragma pack(pop)



#endif // UBLOX_NAV_PVT_HPP