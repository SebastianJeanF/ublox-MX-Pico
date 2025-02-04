#include "../ublox_mx.hpp"
#include "../ublox_nav_pvt.hpp"
#include "tusb.h"
#include <vector>

#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13

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

    gnss_data_t data = {0};

    const int WAIT_TIME_MS = 1000;

    // 200 is good, takes 20ms to retrieve data
    // Wait time of 2000 cuases a lot of waste bytes to be read (~400), ~60ms to read with high SD

    while (true) {

        if (gps.read_GGA_data(&data)) {

            printf("UTC Time: %d\n", data.utc_time);
            printf("Latitude: %f\n", data.latitude);
            printf("Longitude: %f\n", data.longitude);
            printf("Num Satellites: %d\n", data.num_satellites);
        }
        sleep_ms(WAIT_TIME_MS);
    }
}

void parse_PVT(UbxNavPvt *pvt) {

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
}

void demo_PVT() {

    const int WAIT_TIME_MS = 1000;

    gps.begin_PVT(WAIT_TIME_MS);

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
#ifdef UBLOX_TIME
        else {
            absolute_time_t end_time = get_absolute_time();
            int64_t elapsed_us = absolute_time_diff_us(start_time, end_time);
            printf("Elapsed time for attempted read: %lld microseconds\n", elapsed_us);
        }
#endif

        sleep_ms(WAIT_TIME_MS);
    }
}
