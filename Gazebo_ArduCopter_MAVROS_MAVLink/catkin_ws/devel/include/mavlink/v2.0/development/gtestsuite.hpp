/** @file
 *	@brief MAVLink comm testsuite protocol generated from development.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "development.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(development, PARAM_ACK_TRANSACTION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::PARAM_ACK_TRANSACTION packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.param_id = to_char_array("GHIJKLMNOPQRSTU");
    packet_in.param_value = 17.0;
    packet_in.param_type = 199;
    packet_in.param_result = 10;

    mavlink::development::msg::PARAM_ACK_TRANSACTION packet1{};
    mavlink::development::msg::PARAM_ACK_TRANSACTION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
    EXPECT_EQ(packet1.param_result, packet2.param_result);
}

#ifdef TEST_INTEROP
TEST(development_interop, PARAM_ACK_TRANSACTION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_ack_transaction_t packet_c {
         17.0, 17, 84, "GHIJKLMNOPQRSTU", 199, 10
    };

    mavlink::development::msg::PARAM_ACK_TRANSACTION packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.param_id = to_char_array("GHIJKLMNOPQRSTU");
    packet_in.param_value = 17.0;
    packet_in.param_type = 199;
    packet_in.param_result = 10;

    mavlink::development::msg::PARAM_ACK_TRANSACTION packet2{};

    mavlink_msg_param_ack_transaction_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);
    EXPECT_EQ(packet_in.param_result, packet2.param_result);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, AIRSPEED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::AIRSPEED packet_in{};
    packet_in.id = 163;
    packet_in.airspeed = 17.0;
    packet_in.temperature = 17651;
    packet_in.raw_press = 45.0;
    packet_in.flags = 230;

    mavlink::development::msg::AIRSPEED packet1{};
    mavlink::development::msg::AIRSPEED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.airspeed, packet2.airspeed);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.raw_press, packet2.raw_press);
    EXPECT_EQ(packet1.flags, packet2.flags);
}

#ifdef TEST_INTEROP
TEST(development_interop, AIRSPEED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_airspeed_t packet_c {
         17.0, 45.0, 17651, 163, 230
    };

    mavlink::development::msg::AIRSPEED packet_in{};
    packet_in.id = 163;
    packet_in.airspeed = 17.0;
    packet_in.temperature = 17651;
    packet_in.raw_press = 45.0;
    packet_in.flags = 230;

    mavlink::development::msg::AIRSPEED packet2{};

    mavlink_msg_airspeed_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.airspeed, packet2.airspeed);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.raw_press, packet2.raw_press);
    EXPECT_EQ(packet_in.flags, packet2.flags);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, WIFI_NETWORK_INFO)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::WIFI_NETWORK_INFO packet_in{};
    packet_in.ssid = to_char_array("CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG");
    packet_in.channel_id = 235;
    packet_in.signal_quality = 46;
    packet_in.data_rate = 17235;
    packet_in.security = 113;

    mavlink::development::msg::WIFI_NETWORK_INFO packet1{};
    mavlink::development::msg::WIFI_NETWORK_INFO packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.ssid, packet2.ssid);
    EXPECT_EQ(packet1.channel_id, packet2.channel_id);
    EXPECT_EQ(packet1.signal_quality, packet2.signal_quality);
    EXPECT_EQ(packet1.data_rate, packet2.data_rate);
    EXPECT_EQ(packet1.security, packet2.security);
}

#ifdef TEST_INTEROP
TEST(development_interop, WIFI_NETWORK_INFO)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_wifi_network_info_t packet_c {
         17235, "CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG", 235, 46, 113
    };

    mavlink::development::msg::WIFI_NETWORK_INFO packet_in{};
    packet_in.ssid = to_char_array("CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG");
    packet_in.channel_id = 235;
    packet_in.signal_quality = 46;
    packet_in.data_rate = 17235;
    packet_in.security = 113;

    mavlink::development::msg::WIFI_NETWORK_INFO packet2{};

    mavlink_msg_wifi_network_info_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.ssid, packet2.ssid);
    EXPECT_EQ(packet_in.channel_id, packet2.channel_id);
    EXPECT_EQ(packet_in.signal_quality, packet2.signal_quality);
    EXPECT_EQ(packet_in.data_rate, packet2.data_rate);
    EXPECT_EQ(packet_in.security, packet2.security);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, FIGURE_EIGHT_EXECUTION_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::FIGURE_EIGHT_EXECUTION_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.major_radius = 73.0;
    packet_in.minor_radius = 101.0;
    packet_in.orientation = 129.0;
    packet_in.frame = 101;
    packet_in.x = 963498504;
    packet_in.y = 963498712;
    packet_in.z = 213.0;

    mavlink::development::msg::FIGURE_EIGHT_EXECUTION_STATUS packet1{};
    mavlink::development::msg::FIGURE_EIGHT_EXECUTION_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.major_radius, packet2.major_radius);
    EXPECT_EQ(packet1.minor_radius, packet2.minor_radius);
    EXPECT_EQ(packet1.orientation, packet2.orientation);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(development_interop, FIGURE_EIGHT_EXECUTION_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_figure_eight_execution_status_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 963498504, 963498712, 213.0, 101
    };

    mavlink::development::msg::FIGURE_EIGHT_EXECUTION_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.major_radius = 73.0;
    packet_in.minor_radius = 101.0;
    packet_in.orientation = 129.0;
    packet_in.frame = 101;
    packet_in.x = 963498504;
    packet_in.y = 963498712;
    packet_in.z = 213.0;

    mavlink::development::msg::FIGURE_EIGHT_EXECUTION_STATUS packet2{};

    mavlink_msg_figure_eight_execution_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.major_radius, packet2.major_radius);
    EXPECT_EQ(packet_in.minor_radius, packet2.minor_radius);
    EXPECT_EQ(packet_in.orientation, packet2.orientation);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, BATTERY_STATUS_V2)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::BATTERY_STATUS_V2 packet_in{};
    packet_in.id = 199;
    packet_in.temperature = 18275;
    packet_in.voltage = 17.0;
    packet_in.current = 45.0;
    packet_in.capacity_consumed = 73.0;
    packet_in.capacity_remaining = 101.0;
    packet_in.percent_remaining = 10;
    packet_in.status_flags = 963498296;

    mavlink::development::msg::BATTERY_STATUS_V2 packet1{};
    mavlink::development::msg::BATTERY_STATUS_V2 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.voltage, packet2.voltage);
    EXPECT_EQ(packet1.current, packet2.current);
    EXPECT_EQ(packet1.capacity_consumed, packet2.capacity_consumed);
    EXPECT_EQ(packet1.capacity_remaining, packet2.capacity_remaining);
    EXPECT_EQ(packet1.percent_remaining, packet2.percent_remaining);
    EXPECT_EQ(packet1.status_flags, packet2.status_flags);
}

#ifdef TEST_INTEROP
TEST(development_interop, BATTERY_STATUS_V2)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_battery_status_v2_t packet_c {
         17.0, 45.0, 73.0, 101.0, 963498296, 18275, 199, 10
    };

    mavlink::development::msg::BATTERY_STATUS_V2 packet_in{};
    packet_in.id = 199;
    packet_in.temperature = 18275;
    packet_in.voltage = 17.0;
    packet_in.current = 45.0;
    packet_in.capacity_consumed = 73.0;
    packet_in.capacity_remaining = 101.0;
    packet_in.percent_remaining = 10;
    packet_in.status_flags = 963498296;

    mavlink::development::msg::BATTERY_STATUS_V2 packet2{};

    mavlink_msg_battery_status_v2_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.voltage, packet2.voltage);
    EXPECT_EQ(packet_in.current, packet2.current);
    EXPECT_EQ(packet_in.capacity_consumed, packet2.capacity_consumed);
    EXPECT_EQ(packet_in.capacity_remaining, packet2.capacity_remaining);
    EXPECT_EQ(packet_in.percent_remaining, packet2.percent_remaining);
    EXPECT_EQ(packet_in.status_flags, packet2.status_flags);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, COMPONENT_INFORMATION_BASIC)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::COMPONENT_INFORMATION_BASIC packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.capabilities = 93372036854775807ULL;
    packet_in.vendor_name = to_char_array("MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ");
    packet_in.model_name = to_char_array("STUVWXYZABCDEFGHIJKLMNOPQRSTUVW");
    packet_in.software_version = to_char_array("YZABCDEFGHIJKLMNOPQRSTU");
    packet_in.hardware_version = to_char_array("WXYZABCDEFGHIJKLMNOPQRS");
    packet_in.serial_number = to_char_array("UVWXYZABCDEFGHIJKLMNOPQRSTUVWXY");

    mavlink::development::msg::COMPONENT_INFORMATION_BASIC packet1{};
    mavlink::development::msg::COMPONENT_INFORMATION_BASIC packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.capabilities, packet2.capabilities);
    EXPECT_EQ(packet1.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet1.model_name, packet2.model_name);
    EXPECT_EQ(packet1.software_version, packet2.software_version);
    EXPECT_EQ(packet1.hardware_version, packet2.hardware_version);
    EXPECT_EQ(packet1.serial_number, packet2.serial_number);
}

#ifdef TEST_INTEROP
TEST(development_interop, COMPONENT_INFORMATION_BASIC)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_component_information_basic_t packet_c {
         93372036854775807ULL, 963497880, "MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ", "STUVWXYZABCDEFGHIJKLMNOPQRSTUVW", "YZABCDEFGHIJKLMNOPQRSTU", "WXYZABCDEFGHIJKLMNOPQRS", "UVWXYZABCDEFGHIJKLMNOPQRSTUVWXY"
    };

    mavlink::development::msg::COMPONENT_INFORMATION_BASIC packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.capabilities = 93372036854775807ULL;
    packet_in.vendor_name = to_char_array("MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ");
    packet_in.model_name = to_char_array("STUVWXYZABCDEFGHIJKLMNOPQRSTUVW");
    packet_in.software_version = to_char_array("YZABCDEFGHIJKLMNOPQRSTU");
    packet_in.hardware_version = to_char_array("WXYZABCDEFGHIJKLMNOPQRS");
    packet_in.serial_number = to_char_array("UVWXYZABCDEFGHIJKLMNOPQRSTUVWXY");

    mavlink::development::msg::COMPONENT_INFORMATION_BASIC packet2{};

    mavlink_msg_component_information_basic_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.capabilities, packet2.capabilities);
    EXPECT_EQ(packet_in.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet_in.model_name, packet2.model_name);
    EXPECT_EQ(packet_in.software_version, packet2.software_version);
    EXPECT_EQ(packet_in.hardware_version, packet2.hardware_version);
    EXPECT_EQ(packet_in.serial_number, packet2.serial_number);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, GROUP_START)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::GROUP_START packet_in{};
    packet_in.group_id = 963497880;
    packet_in.mission_checksum = 963498088;
    packet_in.time_usec = 93372036854775807ULL;

    mavlink::development::msg::GROUP_START packet1{};
    mavlink::development::msg::GROUP_START packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.group_id, packet2.group_id);
    EXPECT_EQ(packet1.mission_checksum, packet2.mission_checksum);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
}

#ifdef TEST_INTEROP
TEST(development_interop, GROUP_START)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_group_start_t packet_c {
         93372036854775807ULL, 963497880, 963498088
    };

    mavlink::development::msg::GROUP_START packet_in{};
    packet_in.group_id = 963497880;
    packet_in.mission_checksum = 963498088;
    packet_in.time_usec = 93372036854775807ULL;

    mavlink::development::msg::GROUP_START packet2{};

    mavlink_msg_group_start_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.group_id, packet2.group_id);
    EXPECT_EQ(packet_in.mission_checksum, packet2.mission_checksum);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, GROUP_END)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::GROUP_END packet_in{};
    packet_in.group_id = 963497880;
    packet_in.mission_checksum = 963498088;
    packet_in.time_usec = 93372036854775807ULL;

    mavlink::development::msg::GROUP_END packet1{};
    mavlink::development::msg::GROUP_END packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.group_id, packet2.group_id);
    EXPECT_EQ(packet1.mission_checksum, packet2.mission_checksum);
    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
}

#ifdef TEST_INTEROP
TEST(development_interop, GROUP_END)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_group_end_t packet_c {
         93372036854775807ULL, 963497880, 963498088
    };

    mavlink::development::msg::GROUP_END packet_in{};
    packet_in.group_id = 963497880;
    packet_in.mission_checksum = 963498088;
    packet_in.time_usec = 93372036854775807ULL;

    mavlink::development::msg::GROUP_END packet2{};

    mavlink_msg_group_end_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.group_id, packet2.group_id);
    EXPECT_EQ(packet_in.mission_checksum, packet2.mission_checksum);
    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, AVAILABLE_MODES)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::AVAILABLE_MODES packet_in{};
    packet_in.number_modes = 29;
    packet_in.mode_index = 96;
    packet_in.standard_mode = 163;
    packet_in.custom_mode = 963497464;
    packet_in.properties = 963497672;
    packet_in.mode_name = to_char_array("LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS");

    mavlink::development::msg::AVAILABLE_MODES packet1{};
    mavlink::development::msg::AVAILABLE_MODES packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.number_modes, packet2.number_modes);
    EXPECT_EQ(packet1.mode_index, packet2.mode_index);
    EXPECT_EQ(packet1.standard_mode, packet2.standard_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.properties, packet2.properties);
    EXPECT_EQ(packet1.mode_name, packet2.mode_name);
}

#ifdef TEST_INTEROP
TEST(development_interop, AVAILABLE_MODES)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_available_modes_t packet_c {
         963497464, 963497672, 29, 96, 163, "LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS"
    };

    mavlink::development::msg::AVAILABLE_MODES packet_in{};
    packet_in.number_modes = 29;
    packet_in.mode_index = 96;
    packet_in.standard_mode = 163;
    packet_in.custom_mode = 963497464;
    packet_in.properties = 963497672;
    packet_in.mode_name = to_char_array("LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS");

    mavlink::development::msg::AVAILABLE_MODES packet2{};

    mavlink_msg_available_modes_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.number_modes, packet2.number_modes);
    EXPECT_EQ(packet_in.mode_index, packet2.mode_index);
    EXPECT_EQ(packet_in.standard_mode, packet2.standard_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.properties, packet2.properties);
    EXPECT_EQ(packet_in.mode_name, packet2.mode_name);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, CURRENT_MODE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::CURRENT_MODE packet_in{};
    packet_in.standard_mode = 29;
    packet_in.custom_mode = 963497464;
    packet_in.intended_custom_mode = 963497672;

    mavlink::development::msg::CURRENT_MODE packet1{};
    mavlink::development::msg::CURRENT_MODE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.standard_mode, packet2.standard_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.intended_custom_mode, packet2.intended_custom_mode);
}

#ifdef TEST_INTEROP
TEST(development_interop, CURRENT_MODE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_current_mode_t packet_c {
         963497464, 963497672, 29
    };

    mavlink::development::msg::CURRENT_MODE packet_in{};
    packet_in.standard_mode = 29;
    packet_in.custom_mode = 963497464;
    packet_in.intended_custom_mode = 963497672;

    mavlink::development::msg::CURRENT_MODE packet2{};

    mavlink_msg_current_mode_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.standard_mode, packet2.standard_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.intended_custom_mode, packet2.intended_custom_mode);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, AVAILABLE_MODES_MONITOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::AVAILABLE_MODES_MONITOR packet_in{};
    packet_in.seq = 5;

    mavlink::development::msg::AVAILABLE_MODES_MONITOR packet1{};
    mavlink::development::msg::AVAILABLE_MODES_MONITOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(development_interop, AVAILABLE_MODES_MONITOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_available_modes_monitor_t packet_c {
         5
    };

    mavlink::development::msg::AVAILABLE_MODES_MONITOR packet_in{};
    packet_in.seq = 5;

    mavlink::development::msg::AVAILABLE_MODES_MONITOR packet2{};

    mavlink_msg_available_modes_monitor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, TARGET_ABSOLUTE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::TARGET_ABSOLUTE packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.id = 61;
    packet_in.sensor_capabilities = 128;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 129.0;
    packet_in.vel = {{ 157.0, 158.0, 159.0 }};
    packet_in.acc = {{ 241.0, 242.0, 243.0 }};
    packet_in.q_target = {{ 325.0, 326.0, 327.0, 328.0 }};
    packet_in.rates = {{ 437.0, 438.0, 439.0 }};
    packet_in.position_std = {{ 521.0, 522.0 }};
    packet_in.vel_std = {{ 577.0, 578.0, 579.0 }};
    packet_in.acc_std = {{ 661.0, 662.0, 663.0 }};

    mavlink::development::msg::TARGET_ABSOLUTE packet1{};
    mavlink::development::msg::TARGET_ABSOLUTE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.sensor_capabilities, packet2.sensor_capabilities);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.vel, packet2.vel);
    EXPECT_EQ(packet1.acc, packet2.acc);
    EXPECT_EQ(packet1.q_target, packet2.q_target);
    EXPECT_EQ(packet1.rates, packet2.rates);
    EXPECT_EQ(packet1.position_std, packet2.position_std);
    EXPECT_EQ(packet1.vel_std, packet2.vel_std);
    EXPECT_EQ(packet1.acc_std, packet2.acc_std);
}

#ifdef TEST_INTEROP
TEST(development_interop, TARGET_ABSOLUTE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_target_absolute_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 129.0, { 157.0, 158.0, 159.0 }, { 241.0, 242.0, 243.0 }, { 325.0, 326.0, 327.0, 328.0 }, { 437.0, 438.0, 439.0 }, { 521.0, 522.0 }, { 577.0, 578.0, 579.0 }, { 661.0, 662.0, 663.0 }, 61, 128
    };

    mavlink::development::msg::TARGET_ABSOLUTE packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.id = 61;
    packet_in.sensor_capabilities = 128;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 129.0;
    packet_in.vel = {{ 157.0, 158.0, 159.0 }};
    packet_in.acc = {{ 241.0, 242.0, 243.0 }};
    packet_in.q_target = {{ 325.0, 326.0, 327.0, 328.0 }};
    packet_in.rates = {{ 437.0, 438.0, 439.0 }};
    packet_in.position_std = {{ 521.0, 522.0 }};
    packet_in.vel_std = {{ 577.0, 578.0, 579.0 }};
    packet_in.acc_std = {{ 661.0, 662.0, 663.0 }};

    mavlink::development::msg::TARGET_ABSOLUTE packet2{};

    mavlink_msg_target_absolute_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.sensor_capabilities, packet2.sensor_capabilities);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.vel, packet2.vel);
    EXPECT_EQ(packet_in.acc, packet2.acc);
    EXPECT_EQ(packet_in.q_target, packet2.q_target);
    EXPECT_EQ(packet_in.rates, packet2.rates);
    EXPECT_EQ(packet_in.position_std, packet2.position_std);
    EXPECT_EQ(packet_in.vel_std, packet2.vel_std);
    EXPECT_EQ(packet_in.acc_std, packet2.acc_std);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(development, TARGET_RELATIVE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::TARGET_RELATIVE packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.id = 209;
    packet_in.frame = 20;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.pos_std = {{ 157.0, 158.0, 159.0 }};
    packet_in.yaw_std = 241.0;
    packet_in.q_target = {{ 269.0, 270.0, 271.0, 272.0 }};
    packet_in.q_sensor = {{ 381.0, 382.0, 383.0, 384.0 }};
    packet_in.type = 87;

    mavlink::development::msg::TARGET_RELATIVE packet1{};
    mavlink::development::msg::TARGET_RELATIVE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.pos_std, packet2.pos_std);
    EXPECT_EQ(packet1.yaw_std, packet2.yaw_std);
    EXPECT_EQ(packet1.q_target, packet2.q_target);
    EXPECT_EQ(packet1.q_sensor, packet2.q_sensor);
    EXPECT_EQ(packet1.type, packet2.type);
}

#ifdef TEST_INTEROP
TEST(development_interop, TARGET_RELATIVE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_target_relative_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, { 157.0, 158.0, 159.0 }, 241.0, { 269.0, 270.0, 271.0, 272.0 }, { 381.0, 382.0, 383.0, 384.0 }, 209, 20, 87
    };

    mavlink::development::msg::TARGET_RELATIVE packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.id = 209;
    packet_in.frame = 20;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.pos_std = {{ 157.0, 158.0, 159.0 }};
    packet_in.yaw_std = 241.0;
    packet_in.q_target = {{ 269.0, 270.0, 271.0, 272.0 }};
    packet_in.q_sensor = {{ 381.0, 382.0, 383.0, 384.0 }};
    packet_in.type = 87;

    mavlink::development::msg::TARGET_RELATIVE packet2{};

    mavlink_msg_target_relative_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.pos_std, packet2.pos_std);
    EXPECT_EQ(packet_in.yaw_std, packet2.yaw_std);
    EXPECT_EQ(packet_in.q_target, packet2.q_target);
    EXPECT_EQ(packet_in.q_sensor, packet2.q_sensor);
    EXPECT_EQ(packet_in.type, packet2.type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
