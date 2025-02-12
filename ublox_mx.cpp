#include "ublox_mx.hpp"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifdef UBLOX_VERBOSE
#define debug_log(...)       \
    do {                     \
        printf(__VA_ARGS__); \
    } while (0)
#else
#define debug_log(...) \
    do {               \
    } while (0)
#endif

GNSS::GNSS(i2c_inst_t *i2c_type) {
    i2c = i2c_type;
}

// TODO: Abstract to any NMEA message
bool GNSS::parse_gngga(gnss_data_t *data) {

    // Talker ID (Indicating to use any combination of GNSS)
    const std::string TALKER_ID = "GN";

    // Sentence ID (GGA)
    debug_log("Raw Data: %s\n", raw_data.c_str());
#ifdef UBLOX_VERBOSE
    printf("Raw Data: ");
    for (size_t i = 0; i < raw_data.length(); i++) {
        printf("%c", raw_data[i]);
    }
    printf("\n\n\n");
#endif

    size_t start = raw_data.find("$GNGGA");
    if (start == std::string::npos) {
        return false;
    }

    size_t end = raw_data.find('\n', start);
    if (end == std::string::npos) {
        end = raw_data.length();
    }

    std::string sentence = raw_data.substr(start, end - start);

    std::istringstream ss(sentence);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }

    debug_log("GNGGA Sentence: %s\n", raw_data.c_str());

    if (tokens.size() < 7) {
        debug_log("Error: Incomplete GNGGA message: %s\n", sentence.c_str());
        return false;
    }

    // UTC Time
    if (!tokens[1].empty()) {
        char *end;
        double raw_utc = std::strtod(tokens[1].c_str(), &end);
        if (*end != '\0') {
            debug_log("Error: Invalid UTC time value: %s\n", tokens[2].c_str());
            return false;
        }
        data->utc_time = static_cast<uint32_t>(raw_utc);
    } else {
        debug_log("Error: Missing UTC time data in GNGGA message\n");
        return false;
    }

    // Latitude
    if (!tokens[2].empty() && !tokens[3].empty()) {
        char *end;
        double raw_lat = std::strtod(tokens[2].c_str(), &end);
        if (*end != '\0') {
            debug_log("Error: Invalid latitude value: %s\n", tokens[2].c_str());
            return false;
        }

        char lat_dir = tokens[3][0];
        int lat_degrees = static_cast<int>(raw_lat / 100);
        double lat_minutes = raw_lat - (lat_degrees * 100);
        data->latitude = lat_degrees + (lat_minutes / 60.0);

        if (lat_dir == 'S') {
            data->latitude = -data->latitude;
        }
    } else {
        debug_log("Error: Missing latitude data in GNGGA message\n");
        return false;
    }

    // Longitude
    if (!tokens[4].empty() && !tokens[5].empty()) {
        char *end;
        double raw_lon = std::strtod(tokens[4].c_str(), &end);
        if (*end != '\0') {
            debug_log("Error: Invalid longitude value: %s\n", tokens[4].c_str());
            return false;
        }

        char lon_dir = tokens[5][0];
        int lon_degrees = static_cast<int>(raw_lon / 100);
        double lon_minutes = raw_lon - (lon_degrees * 100);
        data->longitude = lon_degrees + (lon_minutes / 60.0);

        if (lon_dir == 'W') {
            data->longitude = -data->longitude;
        }
    } else {
        debug_log("Error: Missing longitude data in GNGGA message\n");
        return false;
    }

    // Number of Satellites
    if (!tokens[7].empty()) {
        char *end;
        long satellites = std::strtol(tokens[7].c_str(), &end, 10);
        if (*end != '\0') {
            debug_log("Error: Invalid satellite count: %s\n", tokens[7].c_str());
            return false;
        }
        data->num_satellites = static_cast<int>(satellites);
    } else {
        debug_log("Error: Missing satellite count in GNGGA message\n");
        return false;
    }

    return true;
}

void print_buffer(uint8_t *buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n\n");
}

void GNSS::request_version() {

    UBXFrame frame;
    frame.msg_class = 0x0a;
    frame.msg_id = 0x04;
    frame.len = 0x00;
    frame.payload = nullptr;

    send_ublox_frame(&frame);

    delete[] frame.payload;
}

// TODO: Optimize this
bool GNSS::read_GGA_data(gnss_data_t *data) {

    // Wait until a '$' is found.
    uint8_t byte;
    int i = 0;
    int buf_max = 2048;
    uint8_t buffer[buf_max];

    while (true) {
        bool success = i2c_read_blocking(i2c, UBLOX_ADDR, buffer + i, 1, false);
        if (!success) continue;
        if (buffer[i] == '$') {
#ifdef UBLOX_VERBOSE
            printf("Found the start of the sentence. Here was right below it\n: ");
            print_buffer(buffer, i);
            printf("\n");
#endif
            break;
        }

        i++;
        if (i == buf_max) {
#ifdef UBLOX_VERBOSE
            printf("Buffer is full. Here is the buffer\n");
            print_buffer(buffer, i);
            printf("\n");
#endif
            i = 0;

            return false;
        }
    }

    // Start building the sentence.
    std::string sentence;
    sentence.push_back('$');

    // Read subsequent bytes until newline is encountered.
    while (true) {
        bool success = i2c_read_blocking(i2c, UBLOX_ADDR, &byte, 1, false);
        if (!success) continue;
        sentence.push_back(byte);
        if (byte == '\n') {

            int i = 0;
            uint8_t buffer2[100];
            while (i < 100) {
                bool success = i2c_read_blocking(i2c, UBLOX_ADDR, buffer2 + i, 1, false);
                if (!success) continue;
                if (buffer2[i] == '\n') {
                    i++;
                    break;
                }
                i++;
            }

            break;
        }
    }

    raw_data = sentence;

    if (raw_data.find("$GNGGA") == std::string::npos) {
        return false;
    }

    debug_log("Bytes Read: %d\n", i);
    debug_log("Received GNSS Data: %s\n", raw_data.c_str());

    data->valid = parse_gngga(data);
    return data->valid;
}

/**
 * Inserts a hexadecimal number into a byte array in little-endian format.
 *
 * @param number      The number to be inserted.
 * @param payload     The byte array where the number will be stored.
 * @param byte_count  The number of bytes to write (e.g., 2 for 16-bit, 4 for 32-bit).
 */
void insert_little_endian(int number, uint8_t *payload, size_t byte_count) {
    // Example: 2000, or 0x07D0
    // payload[i] = 0xD0; // LSB of the value
    // payload[i + 1] = 0x07; // MSB of the value

    const int BITS_PER_BYTE = 8;
    for (size_t i = 0; i < byte_count; i++) {
        payload[i] = (number >> (BITS_PER_BYTE * i)) & 0xFF;
    }
}

std::string addNmeaChecksum(const std::string &pubx) {
    uint8_t calcChecksum = 0;

    for (uint8_t i = 1; i < pubx.size(); i++) {
        calcChecksum = calcChecksum ^ pubx[i];
    }

    //"Extracts" the first four bits and shifts them 4 bits to the right. Bitwise AND followed by a bitshift
    uint8_t nibble1 = (calcChecksum & 0xF0) >> 4;
    uint8_t nibble2 = calcChecksum & 0x0F;

    uint8_t translatedByte1 = (nibble1 <= 0x9) ? (nibble1 + '0') : (nibble1 - 10 + 'A'); // Converting the number "nibble1" into the ASCII representation of that number
    uint8_t translatedByte2 = (nibble2 <= 0x9) ? (nibble2 + '0') : (nibble2 - 10 + 'A'); // Converting the number "nibble2" into the ASCII representation of that number

    return pubx + "*" + (char)translatedByte1 + (char)translatedByte2 + "\r\n";
}

// Function to construct RATE (PUBX,40) command for enabling/disabling NMEA messages
std::string constructPUBX40Message(const std::string &msgId, uint8_t i2cRate, uint8_t otherRates = 0) {
    char checksum[4]; // Increased size to be safe
    std::string pubx = "$PUBX,40," + msgId + "," +
                       std::to_string(i2cRate) + "," +
                       std::to_string(otherRates) + "," +
                       std::to_string(otherRates) + "," +
                       std::to_string(otherRates) + "," +
                       std::to_string(otherRates) + ",0";

    uint8_t ckA = 0, ckB = 0;
    std::string data = pubx.substr(1); // Exclude the '$'
    for (char c : data) {
        ckA += static_cast<uint8_t>(c);
        ckB += ckA;
    }
    sprintf(checksum, "*%02X", (ckA & 0xFF) ^ (ckB & 0xFF)); // Example checksum calculation

    std::string full_message = addNmeaChecksum(pubx);

    debug_log("Full PUBX message: %s\n", full_message.c_str());

    return full_message;
}

// Function to calculate the UBX checksum
void calculateChecksum(const std::vector<uint8_t> &message, uint8_t &ckA, uint8_t &ckB) {
    ckA = 0;
    ckB = 0;
    for (size_t i = 2; i < message.size(); ++i) { // Exclude sync bytes
        ckA += message[i];
        ckB += ckA;
    }
}

std::vector<uint8_t> constructUbxCfgRateMessage(uint16_t measRate, uint16_t navRate, uint16_t timeRef) {
    std::vector<uint8_t> message;

    // UBX message structure
    message.push_back(0xB5); // Sync char 1
    message.push_back(0x62); // Sync char 2
    message.push_back(0x06); // Msg Class: CFG
    message.push_back(0x08); // Msg ID: RATE
    message.push_back(0x06); // Length (LSB): Payload is 6 bytes
    message.push_back(0x00); // Length (MSB)

    // Payload
    message.push_back(static_cast<uint8_t>(measRate & 0xFF));     // Measurement Rate (LSB)
    message.push_back(static_cast<uint8_t>((measRate >> 8) & 0xFF)); // Measurement Rate (MSB)
    message.push_back(static_cast<uint8_t>(navRate & 0xFF));      // Navigation Rate (LSB)
    message.push_back(static_cast<uint8_t>((navRate >> 8) & 0xFF));  // Navigation Rate (MSB)
    message.push_back(static_cast<uint8_t>(timeRef & 0xFF));      // Time Reference (LSB)
    message.push_back(static_cast<uint8_t>((timeRef >> 8) & 0xFF));  // Time Reference (MSB)

    // Calculate checksum
    uint8_t ckA, ckB;
    calculateChecksum(message, ckA, ckB);

    // Append checksum to the message
    message.push_back(ckA);
    message.push_back(ckB);

    return message;
}

void GNSS::read_config(const cfg_key_id_t key_id, const read_config_layer_t layer) {
    UBXFrame frame;
    frame.msg_class = 0x06;
    frame.msg_id = 0x8b;
    frame.len = 8;

    frame.payload = new uint8_t[frame.len];

    // Setup header:
    const uint8_t MSG_VERSION = 0x00;
    const uint8_t POSITION = 0x00; // AKA don't skip any key values in output message
    frame.payload[0] = MSG_VERSION;
    frame.payload[1] = layer;
    frame.payload[2] = POSITION;
    frame.payload[3] = POSITION;

    // frame.payload[4..7] stores the key_id (little-endian format)
    insert_little_endian(key_id, frame.payload + GNSS::KEY_ID_SIZE_BYTES, GNSS::KEY_ID_SIZE_BYTES);

    send_ublox_frame(&frame);

    delete[] frame.payload;
}

bool GNSS::set_speed_ms(const int measure_rate_ms) {
    if (measure_rate_ms < MIN_DATA_RATE_MS) {
        return false;
    }
    UBXFrame frame;
    frame.msg_class = 0x06;
    frame.msg_id = 0x8a;
    frame.len = 0x0a; // Size 10
    frame.payload = new uint8_t[frame.len];

    frame.payload[0] = 0x00; // Message Version
    frame.payload[1] = 0x01; // Layer (0x01 is RAM layer)
    frame.payload[2] = 0x00; // reserved0
    frame.payload[3] = 0x00; // reserved0

    // Modifying configuration
    // 0x30210001 is the KEY ID
    frame.payload[4] = 0x01; // Key ID (LSB)
    frame.payload[5] = 0x00;
    frame.payload[6] = 0x21;
    frame.payload[7] = 0x30; // Key ID (MSB)

    // frame[8..9] stores the value (little-endian format)
    const int num_bytes = 2;
    insert_little_endian(measure_rate_ms, frame.payload + 8, num_bytes);

    send_ublox_frame(&frame);

    delete[] frame.payload;

    return true;
}

void GNSS::set_config(const cfg_key_id_t key_id, const int value, const size_t value_len_bytes, const uint8_t set_config_layer) {

    UBXFrame frame;
    frame.msg_class = 0x06;
    frame.msg_id = 0x8a;
    frame.len = 8 + value_len_bytes;

    // The message payload is 4 bytes of header (version, layer, reserved[2]),
    // 4 bytes for the key ID, plus the value bytes.
    frame.payload = new uint8_t[frame.len];

    // Setup header:
    const uint8_t MSG_VERSION = 0x00;
    const uint8_t RESERVED = 0x00;
    frame.payload[0] = MSG_VERSION;
    frame.payload[1] = set_config_layer;
    frame.payload[2] = RESERVED;
    frame.payload[3] = RESERVED;

    const int HEADER_SIZE = 4;
    // frame.payload[4..7] stores the key_id (little-endian format)
    insert_little_endian(static_cast<uint32_t>(key_id), frame.payload + HEADER_SIZE, GNSS::KEY_ID_SIZE_BYTES);

    // frame.payload[8..(8 + value_len_bytes)] stores the key value (little-endian format)
    insert_little_endian(value, frame.payload + HEADER_SIZE + GNSS::KEY_ID_SIZE_BYTES, value_len_bytes);

    // Send the UBX frame.
    send_ublox_frame(&frame);

    // Free allocated memory.
    delete[] frame.payload;
}

void GNSS::calc_ubx_checksum(UBXFrame *frame) {
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    ck_a += frame->msg_class;
    ck_b += ck_a;

    ck_a += frame->msg_id;
    ck_b += ck_a;

    ck_a += (frame->len & 0xFF);
    ck_b += ck_a;

    ck_a += (frame->len >> 8);
    ck_b += ck_a;

    for (int i = 0; i < frame->len; i++) {
        ck_a += frame->payload[i];
        ck_b += ck_a;
    }

    frame->ck_a = ck_a;
    frame->ck_b = ck_b;
}

bool GNSS::send_ublox_frame(UBXFrame *frame) {

    calc_ubx_checksum(frame);

#define MAX_UBX_FRAME_SIZE 100 // TODO: Optimize this size

    // Create buffer
    uint8_t buffer[MAX_UBX_FRAME_SIZE];
    size_t index = 0;

    // Serialize UBXFrame
    buffer[index++] = frame->sync_1;
    buffer[index++] = frame->sync_2;
    buffer[index++] = frame->msg_class;
    buffer[index++] = frame->msg_id;

    // Length in little endian
    buffer[index++] = frame->len & 0xFF;
    buffer[index++] = (frame->len >> 8) & 0xFF;

    // Payload
    if (frame->len > 0 && frame->payload != nullptr) {
        memcpy(&buffer[index], frame->payload, frame->len);
        index += frame->len;
    }

    // Checksum
    buffer[index++] = frame->ck_a;
    buffer[index++] = frame->ck_b;

#ifdef UBLOX_VERBOSE
    // Send via I2C
    printf("Sending UBX frame of length: %d\n", index);
    for (size_t i = 0; i < index; ++i) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
#endif

    int write_result = i2c_write_blocking(i2c, UBLOX_ADDR, buffer, index, false);
    if (write_result < 0) {
        // I2C write failed
        return false;
    }

    return true;
}

void GNSS::disable_non_GGA_messages(bool enable_GGA) {

    // By default, the GNSS receiver outputs all NMEA messages at 1 Hz on all ports
    std::vector<std::string> nmeaMessages = {"GLL", "GSA", "GSV", "RMC", "VTG", "TXT", "GGA"};

    std::string nmeaGGA = "GGA";
    for (const auto msg : nmeaMessages) {
        // Disable all NMEA messages, except GGA if toggled on
        bool msgEnabled = (enable_GGA && msg.compare(nmeaGGA) == 0);
        const std::string tmp = constructPUBX40Message(msg, msgEnabled);
        const char *cmd = tmp.c_str();

        // Send command to the GNSS receiver
        int result = i2c_write_blocking(i2c, UBLOX_ADDR, (const uint8_t *)cmd, strlen(cmd), false);
        if (result < 0) {
            debug_log("Error: I2C write failed for nmeaMessage (%s)\n", msg.c_str());
        } else {
            debug_log("Success: I2C write for nmeaMessage (%s)\n", msg.c_str());
        }
    }
    debug_log("\n");
}

void GNSS::reset() {

    UBXFrame frame;
    frame.msg_class = 0x06;
    frame.msg_id = 0x04;
    frame.len = 4;

    // The message payload is 4 bytes of header (version, layer, reserved[2]),
    // 4 bytes for the key ID, plus the value bytes.
    frame.payload = new uint8_t[frame.len];

    const uint16_t RESET_MASK = 0xFFFF;
    const uint8_t RESET_MODE = 0x00; // Do a Hardware Reset immediately
    const uint8_t RESERVED = 0x00;

    // frame.payload[0..1] stores the reset mask
    memcpy(frame.payload, &RESET_MASK, sizeof(RESET_MASK));
    frame.payload[2] = RESET_MODE;
    frame.payload[3] = RESERVED;

    // Send the UBX frame.
    send_ublox_frame(&frame);

    delete[] frame.payload;
}

// Helper function to parse and print
void parseUbloxNavPvt(const uint8_t *data, UbxNavPvt *pvt) {

    size_t length = 100;
    // Basic sanity checks:
    // 1) At least 98 bytes for a complete UBX NAV-PVT message:
    if (length < 98) {
        std::cerr << "ERROR: Not enough data to parse UBX-NAV-PVT\n";
        return;
    }
    // 2) Check sync, class, ID, length fields
    if (data[0] != 0xB5 || data[1] != 0x62) {
        std::cerr << "ERROR: Invalid UBX sync chars\n";
        return;
    }

    uint8_t cls = data[2];
    uint8_t id = data[3];
    uint16_t payloadLen = data[4] | (data[5] << 8);  // little-endian

    if (cls != 0x01 || id != 0x07) {
        std::cerr << "ERROR: Not a NAV-PVT message!\n";
        return;
    }
    if (payloadLen != 92) {
        std::cerr << "ERROR: NAV-PVT payload length != 92!\n";
        return;
    }

    // 3) The actual NAV-PVT payload begins at offset 6
    //    We can reinterpret as our struct:
    std::memcpy(pvt, data + 6, sizeof(UbxNavPvt));
}

bool GNSS::read_PVT_data(UbxNavPvt *pvt) {
    const int PACKET_SIZE = 100;
    const int HEADER_SIZE = 4;    // UBX header size: 0xB5, 0x62, 0x01, 0x07
    uint8_t packet[PACKET_SIZE];

    // Synchronize: read bytes until we find a valid UBX header.
    int sync_attempts = 0;
    int MAX_SYNC_ATTEMPTS = 100;
    while (sync_attempts < MAX_SYNC_ATTEMPTS) {
        uint8_t byte;
        // Read a single byte
        if (!i2c_read_blocking(i2c, UBLOX_ADDR, &byte, 1, true)) {
            debug_log("Error reading byte during sync.\n");
            return false;
        }
        if (byte == 0xB5) {  // potential start of header
            packet[0] = byte;

            // Read the next 3 bytes to complete the header
            if (!i2c_read_blocking(i2c, UBLOX_ADDR, packet + 1, HEADER_SIZE - 1, true)) {
                debug_log("Error reading header bytes.\n");
                return false;
            }
            // Check for the UBX-NAV-PVT header sequence: 0xB5, 0x62, 0x01, 0x07
            if (packet[1] == 0x62 && packet[2] == 0x01 && packet[3] == 0x07) {
                break;  // Header found and synchronized
            }
        }
        sync_attempts++;
    }

    // We have our header in packet[0..3]. Read the rest of the packet[HEADER_SIZE..(PACKET_SIZE-1)].
    if (!i2c_read_blocking(i2c, UBLOX_ADDR, packet + HEADER_SIZE, PACKET_SIZE - HEADER_SIZE, true)) {
        debug_log("Error reading packet body.\n");
        return false;
    }

    // --- Now, check for any immediately available new packets ---
    while (true) {
        uint8_t header[HEADER_SIZE];
        if (!i2c_read_blocking(i2c, UBLOX_ADDR, header, HEADER_SIZE, true)) {
            // No immediate new data available, so exit the loop.
            break;
        }

        // Check for the UBX-NAV-PVT header sequence: 0xB5, 0x62, 0x01, 0x07
        if (header[0] != 0xB5 || header[1] != 0x62 || header[2] != 0x01 || header[3] != 0x07) {
            break;
        }

        // Replace packet[HEADER_SIZE..(PACKET_SIZE-1)] with updated data
        if (!i2c_read_blocking(i2c, UBLOX_ADDR, packet + HEADER_SIZE, PACKET_SIZE - HEADER_SIZE, true)) {
            break;
        }
    }

#ifdef UBLOX_VERBOSE
    printf("PVT Data: ");
    for (int i = 0; i < PACKET_SIZE; ++i) {
        printf("%02X ", packet[i]);
    }
    printf("\n\n");
#endif

    parseUbloxNavPvt(packet, pvt);
    return true;
}
bool GNSS::begin_GGA(const int data_rate_ms) {
    if (data_rate_ms < MIN_DATA_RATE_MS) {
        return false;
    }

    disable_non_GGA_messages();

    enable_single_GPS_mode();

    if (!set_speed_ms(data_rate_ms)) {
        return false;
    }

    return true;
}
bool GNSS::begin_PVT(const int data_rate_ms) {
    const bool ENABLE_PVT = true;
    const int PVT_LEN_BYTES = 1;

    if (data_rate_ms < 40) {
        return false;
    }

    const bool enable_GGA = false;

    disable_non_GGA_messages(enable_GGA);

    enable_single_GPS_mode();

    if (!set_speed_ms(data_rate_ms)) {
        return false;
    }

    set_config(GNSS::CFG_MSGOUT_UBX_NAV_PVT_I2C, ENABLE_PVT, PVT_LEN_BYTES);

    return true;
}

void GNSS::enable_single_GPS_mode() {
    const bool isEnabled = false;
    const size_t LEN_BYTES = 1;
    // const int WAIT_TIME_MS = 500;

    std::vector<GNSS::cfg_key_id_t> key_ids {GNSS::CFG_SIGNAL_GAL_ENA, GNSS::CFG_SIGNAL_BDS_ENA, GNSS::CFG_SIGNAL_GLO_ENA, GNSS::CFG_SIGNAL_BDS_B1_ENA, GNSS::CFG_SIGNAL_BDS_B1C_ENA, GNSS::CFG_SIGNAL_GAL_E1_ENA, GNSS::CFG_SIGNAL_GLO_L1_ENA};

    for (const auto key_id : key_ids) {
        set_config(key_id, isEnabled, LEN_BYTES);
        // sleep_ms(WAIT_TIME_MS);
    }
}