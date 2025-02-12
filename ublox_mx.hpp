/**
 * @file ublox_mx.hpp
 * @author sj728
 * @author csg83
 *
 * @brief Header file for the GNSS class for the U-blox GNSS receiver (Max M10s)
 */

#ifndef UBLOX_MX_HPP
#define UBLOX_MX_HPP

#include "hardware/i2c.h"
#include "ublox_defs.hpp"
#include "ublox_nav_pvt.hpp"
#include <string>

#define UBLOX_ADDR (0x42)

/**
 * Representation of the U-blox GNSS receiver.
 */
class GNSS {
public:
    /**
     * @brief Layers for setting configuration parameters.
     *
     * These layers indicate where the configuration will be stored:
     * - RAM: Temporary configuration (will be lost on power cycle).
     * - BBR: Battery-backed RAM.
     * - FLASH: Permanent storage.
     *
     * Multiple layers can be confiugred at once too by using the
     * bitwise OR operator. Ex) SET_RAM_LAYER | SET_BBR_LAYER
     */
    enum set_config_layer_t : uint8_t {
        SET_RAM_LAYER = 0x01,
        SET_BBR_LAYER = 0x02,
        SET_FLASH_LAYER = 0x04,
    };

    /**
     * @brief Layers for reading configuration parameters.
     *
     * These layers indicate where the configuration will be read from:
     * - RAM: Temporary configuration (will be lost on power cycle).
     * - BBR: Battery-backed RAM.
     * - FLASH: Permanent storage.
     * - DEFAULT: Default configuration.
     */
    enum read_config_layer_t : uint8_t {
        READ_RAM_LAYER = 0,
        READ_BBR_LAYER = 1,
        READ_FLASH_LAYER = 4,
        READ_DEFAULT_LAYER = 7,
    };

    enum cfg_key_id_t : uint32_t {
        CFG_MSGOUT_UBX_NAV_PVT_I2C = 0x20910006,
        CFG_RATE_MEAS_KEY_ID = 0x30210001,
        CFG_RATE_NAV_KEY_ID = 0x30210002,

        CFG_SIGNAL_GAL_ENA = 0x10310021,
        CFG_SIGNAL_BDS_ENA = 0x10310022,
        CFG_SIGNAL_GLO_ENA = 0x10310025,

        CFG_SIGNAL_SBAS_ENA = 0x10310020,
        CFG_SIGNAL_QZSS_ENA = 0x10310024,

        CFG_SIGNAL_SBAS_L1CA_ENA = 0x10310005,
        CFG_SIGNAL_QZSS_L1CA_ENA = 0x10310012,
        CFG_SIGNAL_QZSS_L1S_ENA = 0x10310014,

        CFG_SIGNAL_GLO_L1_ENA = 0x10310018,
        CFG_SIGNAL_GAL_E1_ENA = 0x10310007,
        CFG_SIGNAL_BDS_B1_ENA = 0x1031000d,
        CFG_SIGNAL_BDS_B1C_ENA = 0x1031000f,
    };

    /**
     * Initializes a U-blox GNSS object on an I2C bus.
     * @param i2c_type The I2C bus that this sensor is on
     */
    GNSS(i2c_inst_t *i2c_type);

    /**
     * Resets the GNSS receiver to default configuration
     * settings by sending a UBX-CFG-RST (0x06 0x04) message
     */
    void reset();

    /**
     * Reads a configuration parameter by sending a UBX-CFG-VALGET (0x06 0x8b) message
     * @param key_id The key ID of the configuration parameter
     * @param layer The memory layer to read the configuration from
     */
    void read_config(cfg_key_id_t key_id, read_config_layer_t layer = READ_RAM_LAYER);

    /**
     * Sets a configuration parameter by sending
     * UBX-CFG-VALSET (0x06 0x8a) command to the receiver
     * @param key_id The key ID of the configuration parameter
     *
     */
    void set_config(cfg_key_id_t key_id, int value, size_t value_len_bytes, uint8_t set_config_layer = SET_RAM_LAYER);

    /**
     * Sets the speed at which the GNSS receiver
     * outputs data by sending a UBX-CFG-VALSET (0x06 0x8a) message,
     * to modify the configuration for measurement rate CFG_RATE_MEAS
     * @param  measure_rate_ms The measurement rate in milliseconds. Cannot
     * be lower than 40ms.
     * @return True on successful transmission, false otherwise
     */
    bool set_speed_ms(int measure_rate_ms);

    /**
     * Disables all NMEA messages except for GGA. By default, the GNSS receiver
     * outputs all NMEA messages at 1 Hz on all ports
     * @param enable_GGA If true, GGA messages will also be disabled
     */
    void disable_non_GGA_messages(bool enable_GGA = true);

    /**
     * Disables all GNSS's in the GNSS receiver except for GPS (all are
     * enabled by default, so they must be manually turned off)
     */
    void enable_single_GPS_mode();

    /**
     * Sets up the GNSS to begin sending PVT data
     * @param data_rate_hz The rate at which the GNSS will send PVT data.
     * Must be between 1 and 25 hz
     * @return True on successful transmission, false otherwise
     */
    bool begin_PVT(int data_rate_hz);

    /**
     * Reads the PVT data from the GNSS receiver
     * @param pvt The PVT data struct to store the data in
     */
    bool read_PVT_data(UbxNavPvt *pvt);

    /**
     * Sets up the GNSS to begin sending NMEA GGA data
     * @param data_rate_hz The rate at which the GNSS will send GGA data.
     * Must be between 1 and 25 hz
     * @return True on successful transmission, false otherwise
     */
    bool begin_GGA(int data_rate_hz);

    /**
     * Reads the GGA data from the GNSS receiver
     * @param data The GNSS data struct to store the data in
     * @return True on successful transmission, false otherwise
     */
    bool read_GGA_data(gnss_data_t *data);

private:
    bool parse_gngga(gnss_data_t *data);

    /**
     * Calculates a frame's UBX checksum, as defined in
     * section 3.4 of the interface description document
     * @param frame The UBX frame
     */
    void calc_ubx_checksum(UBXFrame *frame);

    /**
     * Sends a UBX frame to the GNSS receiver.
     * @param frame The UBX frame to send
     * @return True on successful transmission, false otherwise
     */
    bool send_ublox_frame(UBXFrame *frame);

    /**
     * Sends command to get the version of the GNSS receiver
     */
    void request_version();

    std::string raw_data;

    /**
     * The I2C bus.
     */
    i2c_inst_t *i2c;

    /**
     * The size of the UBX header in bytes
     */
    static const int KEY_ID_SIZE_BYTES = 4;

    /**
     * The minimum data rate in milliseconds that
     * the GNSS receiver can output data at
     */
    static const int MAX_DATA_RATE_HZ = 25;
};

#endif // UBLOX_MX_HPP