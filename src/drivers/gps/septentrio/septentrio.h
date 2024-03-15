/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file septentrio.h
 *
 * Septentrio GNSS receiver driver
 *
 * @author Matej Franceskin <Matej.Franceskin@gmail.com>
 * @author <a href="https://github.com/SeppeG">Seppe Geuens</a>
 * @author Thomas Frans
*/

#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

#include "sbf.h"

/* Message decoder state */
typedef enum {
	SBF_DECODE_SYNC1 = 0,
	SBF_DECODE_SYNC2,
	SBF_DECODE_PAYLOAD,
	SBF_DECODE_RTCM3
} sbf_decode_state_t;

/* Struct for dynamic allocation of satellite info data */
struct GPSSatelliteInfo {
	satellite_info_s _data;
};

enum class SeptentrioDumpCommMode : int32_t {
	Disabled = 0,
	Full, ///< dump full RX and TX data for all devices
	RTCM ///< dump received RTCM from Main GPS
};

enum class SeptentrioGPSResetType {
	/**
	 * There is no pending GPS reset.
	 */
	None,

	/**
	 * In hot start mode, the receiver was powered down only for a short time (4 hours or less),
	 * so that its ephemeris is still valid. Since the receiver doesn't need to download ephemeris
	 * again, this is the fastest startup method.
	 */
	Hot,

	/**
	 * In warm start mode, the receiver has approximate information for time, position, and coarse
	 * satellite position data (Almanac). In this mode, after power-up, the receiver normally needs
	 * to download ephemeris before it can calculate position and velocity data.
	 */
	Warm,

	/**
	 * In cold start mode, the receiver has no information from the last position at startup.
	 * Therefore, the receiver must search the full time and frequency space, and all possible
	 * satellite numbers. If a satellite signal is found, it is tracked to decode the ephemeris,
	 * whereas the other channels continue to search satellites. Once there is a sufficient number
	 * of satellites with valid ephemeris, the receiver can calculate position and velocity data.
	 */
	Cold
};

enum class SeptentrioGPSOutputMode {
	GPS,		/**< Only GPS output */
	GPSAndRTCM,	/**< GPS and RTCM output */
}

class SeptentrioGPS : public ModuleBase<SeptentrioGPS>, public device::Device
{
public:
	SeptentrioGPS(const char *device_path);
	~SeptentrioGPS() override;

	int print_status() override;

	void run() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SeptentrioGPS *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Reset the connected GPS receiver.
	 *
	 * @return 0 on success, <0 on error
	*/
	int reset();
private:
	/**
	 * Schedule a reset of the connected receiver.
	 */
	void schedule_reset(SeptentrioGPSResetType type);

	/**
	 * Configure the receiver.
	 *
	 * @return 0 on success, <0 otherwise
	 */
	int configure();

	/**
	 * Keep processing receiver messages until an error occurs.
	 */
	void process_until_error();

	/**
	 * Open the serial connection to the receiver.
	 * On error, wait a second and return.
	 * Does nothing if the connection is already open.
	 *
	 * @return 0 on success, or -1 on error
	 */
	int serial_open();

	/**
	 * @brief Close the serial connection to the receiver.
	 *
	 * Does nothing if the connection is already closed.
	 *
	 * @return 0 on success or closed connection, -1 on error
	 */
	int serial_close();

	/**
	 * @brief Parse the binary SBF packet.
	 *
	 * @return 0 = decoding, 1 = message handled, 2 = sat info message handled
	 */
	int parse_char(const uint8_t b);

	/**
	 * @brief Add payload rx byte.
	 *
	 * @return -1 = error, 0 = ok, 1 = payload completed
	 */
	int payload_rx_add(const uint8_t b);

	/**
	 * @brief Parses incoming SBF blocks.
	 *
	 * @return 0 = no message handled, 1 = message handled, 2 = sat info message handled
	 */
	int payload_rx_done();

	/**
	 * @brief Reset the parse state machine for a fresh start.
	 */
	void decode_init();

	/**
	 * @brief Send a message.
	 *
	 * @return true on success, false on write error (errno set)
	 */
	bool send_message(const char *msg);

	/**
	 * @brief Send a message and waits for acknowledge
	 *
	 * @return true on success, false on write error (errno set) or ack wait timeout
	 */
	bool send_message_and_wait_for_ack(const char *msg, const int timeout);

	/**
	 * @brief Receive incoming messages
	 *
	 * @return -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
	 */
	int receive(unsigned timeout);

	/**
	 * @brief Write to the receiver
	 *
	 * @param buf 		Data to be written
	 * @param buf_length 	Amount of bytes to be written
	 *
	 * @return the number of bytes written on success, or -1 otherwise
	*/
	int write(const void *buf, int buf_length);

	/**
	 * @brief Dump GPS communication.
	 * @param data message
	 * @param len length of the message
	 * @param mode calling source
	 * @param msg_to_gps_device if true, this is a message sent to the gps device, otherwise it's from the device
	 */
	void dump_gps_data(uint8_t *data, size_t len, SeptentrioDumpCommMode mode, bool msg_to_gps_device)

	// TODO: Document
	void initialize_communication_dump()

	px4::atomic<SeptentrioGPSResetType>		_scheduled_reset{SeptentrioGPSResetType::None};			///< The type of receiver reset that is scheduled
	sbf_decode_state_t				_decode_state{SBF_DECODE_SYNC1};				///< State of the SBF parser
	int						_serial_fd{-1};							///< The file descriptor used for communication with the receiver
	char						_port[20] {};							///< The path of the used serial device
	bool						_configured{false};						///< Configuration status of the connected receiver
	uint64_t 					_last_timestamp_time{0};					/// TODO: Document
	uint8_t 					_msg_status{0};							/// TODO: Document
	uint16_t 					_rx_payload_index{0};						/// TODO: Document
	sbf_buf_t 					_buf;								/// TODO: Document
	OutputMode 					_output_mode{OutputMode::GPS};					/// TODO: Document
	RTCMParsing					*_rtcm_parsing{nullptr};						/// TODO: Document
	satellite_info_s				*_p_report_sat_info{nullptr};					///< Pointer to uORB topic for satellite info
	unsigned					_rate_reading{0}; 						///< Reading rate in B/s
	sensor_gps_s					_report_gps_pos{};						///< uORB topic for gps position
	GPSSatelliteInfo				*_sat_info{nullptr};						///< Instance of GPS sat info data object
	gps_dump_s 					*_dump_to_device{nullptr};					/// TODO: Document
	gps_dump_s 					*_dump_from_device{nullptr};					/// TODO: Document
	uORB::PublicationMulti<sensor_gps_s> 		_report_gps_pos_pub{ORB_ID(sensor_gps)};			///< uORB pub for gps position
	uORB::PublicationMulti<satellite_info_s>	_report_sat_info_pub{ORB_ID(satellite_info)};			///< uORB pub for satellite info
	uORB::PublicationMulti<sensor_gnss_relative_s>	_sensor_gnss_relative_pub{ORB_ID(sensor_gnss_relative)};	/// TODO: Document
};
