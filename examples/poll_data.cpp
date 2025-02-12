/**
 * @file poll_data.cpp
 * @author sj728
 * @author csg83
 *
 * @brief Example program to poll GNSS data from the U-blox GNSS receiver
 */

#include "../ublox_mx.hpp"
#include "../ublox_nav_pvt.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "tusb.h"
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13
const int WAIT_TIME_HZ = 20;

const int HZ_TO_MS = 1000;
const int WAIT_TIME_MS = HZ_TO_MS / WAIT_TIME_HZ;

GNSS gps(I2C_PORT);

void demo_GGA();
void demo_PVT();

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    while (!tud_cdc_connected()) {
        sleep_ms(500);
    }
    printf("Connected\n");

    demo_PVT();
    // demo_GGA();
}

void demo_GGA() {
    if (!gps.begin_GGA(WAIT_TIME_HZ)) return;

    gnss_data_t data = {0};
    int i = 0;
    while (true) {
        if (gps.read_GGA_data(&data)) {
            i++;
            printf("UTC Time: %d\n", data.utc_time);
            printf("Latitude: %f\n", data.latitude);
            printf("Longitude: %f\n", data.longitude);
            printf("Num Satellites: %d\n", data.num_satellites);
            printf("Count: %d\n\n\n", i);
        }
        sleep_ms(WAIT_TIME_MS);
    }
}

void t_parse_PVT(UbxNavPvt *pvt) {

    // Print out some key fields
    double longitudeDeg = pvt->lon * 1e-7;
    double latitudeDeg = pvt->lat * 1e-7;
    double heightEllipsoid = pvt->height / 1000.0; // convert mm to meters
    double heightMSL = pvt->hMSL / 1000.0;
    double groundSpeedMs = pvt->gSpeed / 1000.0;
    double headingDeg = pvt->headMot * 1e-5;  // to degrees

    std::cout << std::dec << std::fixed << std::setprecision(7);
    std::cout << "NAV-PVT:\n";
    std::cout << "  iTOW (ms)  : " << pvt->iTOW << "\n";
    std::cout << "  UTC Date   : " << (int)pvt->year << "-"
              << (int)pvt->month << "-"
              << (int)pvt->day << "\n";
    std::cout << "  UTC Time   : " << (int)pvt->hour << ":"
              << (int)pvt->min << ":"
              << (int)pvt->sec << "\n";
    std::cout << "  fixType    : " << (int)pvt->fixType << " (0=no,1=DR,2=2D,3=3D,4=GNSS+DR,5=time)\n";
    std::cout << "  numSV      : " << (int)pvt->numSV << "\n";
    std::cout << "  Longitude  : " << longitudeDeg << " deg\n";
    std::cout << "  Latitude   : " << latitudeDeg << " deg\n";
    std::cout << "  Height(ell): " << heightEllipsoid << " m\n";
    std::cout << "  Height(MSL): " << heightMSL << " m\n";
    std::cout << "  Ground Spd : " << groundSpeedMs << " m/s\n";
    std::cout << "  Heading    : " << headingDeg << " deg\n";
    std::cout << "  pDOP       : " << pvt->pDOP * 0.01 << "\n";
    std::cout << "\n\n\n";
}

// (Assuming your struct definition is already included from "ublox_nav_pvt.hpp")

void parse_PVT(UbxNavPvt *pvt) {
    // Convert some values to human-friendly units
    double longitudeDeg = pvt->lon * 1e-7;
    double latitudeDeg = pvt->lat * 1e-7;
    double heightEllipsoid = pvt->height / 1000.0; // mm to meters
    double heightMSL = pvt->hMSL / 1000.0;
    double groundSpeedMs = pvt->gSpeed / 1000.0;
    double headingDeg = pvt->headMot * 1e-5;  // 1e-5 degrees
    double headAccDeg = pvt->headAcc * 1e-5;  // 1e-5 degrees
    double magDecDeg = pvt->magDec * 0.01;   // 1e-2 degrees
    double magAccDeg = pvt->magAcc * 0.01;   // 1e-2 degrees

    // Set up formatting for output
    std::cout << std::dec << std::fixed << std::setprecision(7);
    std::cout << "NAV-PVT:\n";
    std::cout << "  iTOW (ms)  : " << pvt->iTOW << "\n";
    std::cout << "  UTC Date   : " << (int)pvt->year << "-"
              << (int)pvt->month << "-"
              << (int)pvt->day << "\n";
    std::cout << "  UTC Time   : " << (int)pvt->hour << ":"
              << (int)pvt->min << ":"
              << (int)pvt->sec << "\n";

    // Additional time and validity information
    std::cout << "  valid      : " << (int)pvt->valid << "\n";
    std::cout << "  tAcc       : " << pvt->tAcc << " ns\n";
    std::cout << "  nano       : " << pvt->nano << " ns\n";

    // Fix and flag information
    std::cout << "  fixType    : " << (int)pvt->fixType
              << " (0=no fix, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=time)\n";
    std::cout << "  flags      : 0x" << std::hex << (int)pvt->flags << "\n";
    std::cout << "  flags2     : 0x" << std::hex << (int)pvt->flags2 << "\n";
    std::cout << std::dec;  // Return to decimal formatting

    // Satellite information
    std::cout << "  numSV      : " << (int)pvt->numSV << "\n";

    // Position information
    std::cout << "  Longitude  : " << longitudeDeg << " deg\n";
    std::cout << "  Latitude   : " << latitudeDeg << " deg\n";
    std::cout << "  Height(ell): " << heightEllipsoid << " m\n";
    std::cout << "  Height(MSL): " << heightMSL << " m\n";

    // Accuracy information
    std::cout << "  hAcc       : " << pvt->hAcc << " mm\n";
    std::cout << "  vAcc       : " << pvt->vAcc << " mm\n";

    // Velocity information (in mm/s)
    std::cout << "  velN       : " << pvt->velN << " mm/s\n";
    std::cout << "  velE       : " << pvt->velE << " mm/s\n";
    std::cout << "  velD       : " << pvt->velD << " mm/s\n";
    std::cout << "  Ground Spd : " << groundSpeedMs << " m/s\n";

    // Speed and heading accuracy
    std::cout << "  sAcc       : " << pvt->sAcc << " mm/s\n";
    std::cout << "  Heading    : " << headingDeg << " deg\n";
    std::cout << "  headAcc    : " << headAccDeg << " deg\n";

    // Dilution of precision
    std::cout << "  pDOP       : " << pvt->pDOP * 0.01 << "\n";

    // Additional flags and reserved bytes
    std::cout << "  flags3     : 0x" << std::hex << pvt->flags3 << "\n";
    std::cout << std::dec; // back to decimal
    std::cout << "  reserved0  : ";
    for (int i = 0; i < 4; ++i) {
        std::cout << "0x" << std::hex << (int)pvt->reserved0[i] << " ";
    }
    std::cout << std::dec << "\n";

    // Additional heading and magnetic field information
    std::cout << "  headVeh    : " << pvt->headVeh << "\n";
    std::cout << "  magDec     : " << magDecDeg << " deg\n";
    std::cout << "  magAcc     : " << magAccDeg << " deg\n";

    std::cout << "\n\n";
}

void demo_PVT() {

    if (!gps.begin_PVT(WAIT_TIME_HZ)) return;

    UbxNavPvt data = {0};

    while (true) {

#ifdef UBLOX_TIME
        absolute_time_t start_time = get_absolute_time();
#endif

        if (gps.read_PVT_data(&data)) {

#ifdef UBLOX_TIME
            // From measurements, it takes ~3ms to read data (with UBLOX_VERBOSE off)
            absolute_time_t end_time = get_absolute_time();
            int64_t elapsed_us = absolute_time_diff_us(start_time, end_time);
            printf("Elapsed time: %lld microseconds\n", elapsed_us);
#endif

            parse_PVT(&data);
        }

        sleep_ms(WAIT_TIME_MS);
    }
}
