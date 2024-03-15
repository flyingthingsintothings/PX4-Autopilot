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
 * @file septentrio.cpp
 *
 * Septentrio GNSS receiver driver
 *
 * @author Matej Franceskin <Matej.Franceskin@gmail.com>
 * @author <a href="https://github.com/SeppeG">Seppe Geuens</a>
 * @author Thomas Frans
*/

#include <string.h>
#include <ctime>
#include <cmath>

#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>

#include "septentrio.h"
#include "../gps/devices/src/rtcm.h"

#define SBF_CONFIG_TIMEOUT	1000		///< ms, timeout for waiting ACK
#define SBF_PACKET_TIMEOUT	2		///< ms, if now data during this delay assume that full update received
#define DISABLE_MSG_INTERVAL	1000000		///< us, try to disable message with this interval
#define MSG_SIZE		100 		///< size of the message to be sent to the receiver.
#define TIMEOUT_1HZ		1300		///< Timeout time in mS, 1000 mS (1Hz) + 300 mS delta for error
#define RATE_MEASUREMENT_PERIOD 5_s		/// TODO: Document

/**** Trace macros, disable for production builds */
#define SBF_TRACE_PARSER(...)   {/*GPS_INFO(__VA_ARGS__);*/}    /* decoding progress in parse_char() */
#define SBF_TRACE_RXMSG(...)    {/*GPS_INFO(__VA_ARGS__);*/}    /* Rx msgs in payload_rx_done() */
#define SBF_INFO(...)           {GPS_INFO(__VA_ARGS__);}

/**** Warning macros, disable to save memory */
#define SBF_WARN(...)        {GPS_WARN(__VA_ARGS__);}
#define SBF_DEBUG(...)       {/*GPS_WARN(__VA_ARGS__);*/}

// Commands
#define SBF_FORCE_INPUT "SSSSSSSSSS\n"											/**< Force input on the connected port */

#define SBF_CONFIG_RESET_HOT "" \
	SBF_FORCE_INPUT"ExeResetReceiver, soft, none\n"

#define SBF_CONFIG_RESET_WARM "" \
	SBF_FORCE_INPUT"ExeResetReceiver, soft, PVTData\n"

#define SBF_CONFIG_RESET_COLD "" \
	SBF_FORCE_INPUT"ExeResetReceiver, hard, SatData\n"

#define SBF_CONFIG "setSBFOutput, Stream1, %s, PVTGeodetic+VelCovGeodetic+DOP+AttEuler+AttCovEuler, msec100\n"		/**< Configure the correct blocks for GPS positioning and heading */

#define SBF_CONFIG_BAUDRATE "setCOMSettings, %s, baud%d\n"

#define SBF_CONFIG_RESET "setSBFOutput, Stream1, %s, none, off\n"

#define SBF_CONFIG_RECEIVER_DYNAMICS "setReceiverDynamics, %s, UAV\n"

#define SBF_CONFIG_ATTITUDE_OFFSET "setAttitudeOffset, %.3f, %.3f\n"

#define SBF_TX_CFG_PRT_BAUDRATE 115200

#define SBF_DATA_IO "setDataInOut, %s, Auto, SBF\n"

#define SBF_CONFIG_RTCM_STATIC1 "" \
	"setReceiverDynamics, Low, Static\n"

#define SBF_CONFIG_RTCM_STATIC2 "" \
	"setPVTMode, Static, , Geodetic1\n"

#define SBF_CONFIG_RTCM_STATIC_COORDINATES "" \
	"setStaticPosGeodetic, Geodetic1, %f, %f, %f\n"

#define SBF_CONFIG_RTCM_STATIC_OFFSET "" \
	"setAntennaOffset, Main, %f, %f, %f\n"

SeptentrioGPS::SeptentrioGPS(const char* device_path) :
	Device(MODULE_NAME)
{
	strncpy(_port, device_path, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	_report_gps_pos.heading = NAN;
	_report_gps_pos.heading_offset = NAN;

	int32_t enable_sat_info = 0;
	param_get(param_find("SEPT_SAT_INFO"), &enable_sat_info);

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_sat_info = new GPSSatelliteInfo();
		_p_report_sat_info = &_sat_info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}
}

SeptentrioGPS::~SeptentrioGPS()
{
	delete _dump_from_device;
	delete _dump_to_device;
	delete _rtcm_parsing;
	delete _sat_info;
}

int SeptentrioGPS::print_status()
{
	PX4_INFO("status: %s, port: %s", _healthy ? "OK" : "NOT OK", _port);
	PX4_INFO("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");
	PX4_INFO("rate reading: \t\t%6i B/s", _rate_reading);

	if (_report_gps_pos.timestamp != 0) {
		if (_helper) {
			PX4_INFO("rate position: \t\t%6.2f Hz", (double)_helper->getPositionUpdateRate());
			PX4_INFO("rate velocity: \t\t%6.2f Hz", (double)_helper->getVelocityUpdateRate());
		}

		PX4_INFO("rate publication:\t\t%6.2f Hz", (double)_rate);
		PX4_INFO("rate RTCM injection:\t%6.2f Hz", (double)_rate_rtcm_injection);

		print_message(ORB_ID(sensor_gps), _report_gps_pos);
	}

	if (_instance == Instance::Main && _secondary_instance.load()) {
		GPS *secondary_instance = _secondary_instance.load();
		secondary_instance->print_status();
	}

	return 0;
}

void SeptentrioGPS::run()
{
	param_t handle = param_find("SEPT_YAW_OFFS");
	float heading_offset = 0.f;

	if (handle != PARAM_INVALID) {
		param_get(handle, &heading_offset);
		heading_offset = matrix::wrap_pi(math::radians(heading_offset));
	}

	initialize_communication_dump();

	// Set up the communication, configure the receiver and start processing data in a loop.
	while (!should_exit()) {
		if (open_serial() != 0) {
			continue;
		}

		decode_init();

		// If configuration is successful, start processing messages.
		if (configure(heading_offset) == 0) {

			/* reset report */
			memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));
			_report_gps_pos.heading = NAN;
			_report_gps_pos.heading_offset = heading_offset;

			// Read data from the receiver and publish it until an error occurs.
			process_until_error();

			if (_healthy) {
				_healthy = false;
				_rate = 0.0f;
				_rate_rtcm_injection = 0.0f;
			}
		}

		close_serial();
	}

	PX4_INFO("exiting");
}

int SeptentrioGPS::task_spawn(int argc, char *argv[])
{
	static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(2040);

	_task_id = px4_task_spawn_cmd("septentrio",
					SCHED_DEFAULT,
					SCHED_PRIORITY_SLOW_DRIVER,
					TASK_STACK_SIZE,
					&run_trampoline,
					(char *const *)argv);

	if (_task_id < 0) {
		_task_id = 1;
		return -errno;
	}

	return 0;
}

SeptentrioGPS* SeptentrioGPS::instantiate(int argc, char *argv[])
{
	const char* device_path = nullptr;
	SeptentrioGPS* gps = nullptr;
	bool error_flag = false;
	int ch;
	int myoptind = 1;
	const char* myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;
		case '?':
			error_flag = true;
			break;
		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	if (device_path && (access(device_path, R_OK|W_OK) == 0)) {
		gps = new SeptentrioGPS(device_path);
	} else {
		PX4_ERR("invalid device (-d) %s", device_path ? device_path : "");
	}

	return gps;
}

// TODO: Make sure all the required commands are available!
// Called from outside driver thread.
// Returns 0 on success, -1 otherwise.
int SeptentrioGPS::custom_command(int argc, char* argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	SeptentrioGPS *_instance = get_instance();

	bool res = false;

	if (argc == 2 && !strcmp(argv[0], "reset")) {

		if (!strcmp(argv[1], "hot")) {
			res = true;
			_instance->schedule_reset(SeptentrioGPSResetType::Hot);

		} else if (!strcmp(argv[1], "cold")) {
			res = true;
			_instance->schedule_reset(SeptentrioGPSResetType::Cold);

		} else if (!strcmp(argv[1], "warm")) {
			res = true;
			_instance->schedule_reset(SeptentrioGPSResetType::Warm);
		}
	}

	if (res) {
		PX4_INFO("Resetting GPS - %s", argv[1]);
		return 0;
	}

	return (res) ? 0 : print_usage("unknown command");
}

int SeptentrioGPS::print_usage(const char *reason)
{

	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPS driver module that handles the communication with Septentrio devices and publishes the position via uORB.

The module currently only supports a single GPS device.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("SeptentrioGPS", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "GPS device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int SeptentrioGPS::reset()
{
	bool res = false;

	switch (restart_type) {
	case GPSRestartType::Hot:
		res = send_message_and_wait_for_ack(SBF_CONFIG_RESET_HOT, SBF_CONFIG_TIMEOUT);
		break;

	case GPSRestartType::Warm:
		res = send_message_and_wait_for_ack(SBF_CONFIG_RESET_WARM, SBF_CONFIG_TIMEOUT);
		break;

	case GPSRestartType::Cold:
		res = send_message_and_wait_for_ack(SBF_CONFIG_RESET_COLD, SBF_CONFIG_TIMEOUT);
		break;

	default:
		break;
	}

	return (res) ? 0 : -1;
}

void GPS::schedule_reset(SeptentrioGPSResetType reset_type)
{
	_scheduled_reset.store(reset_type);

	if (_instance == Instance::Main && _secondary_instance.load()) {
		GPS *secondary_instance = _secondary_instance.load();
		secondary_instance->schedule_reset(reset_type);
	}
}

int SeptentrioGPS::configure(float heading_offset)
{
	handle = param_find("SEPT_PITCH_OFFS");
	float pitch_offset = 0.f;

	if (handle != PARAM_INVALID) {
		param_get(handle, &pitch_offset);
	}

		_configured = false;

	setBaudrate(SBF_TX_CFG_PRT_BAUDRATE);
	baudrate = SBF_TX_CFG_PRT_BAUDRATE;
	_output_mode = config.output_mode;

	send_message(SBF_CONFIG_FORCE_INPUT);

	// flush input and wait for at least 50 ms silence
	decodeInit();
	receive(50);
	decodeInit();

	char buf[GPS_READ_BUFFER_SIZE];
	char com_port[5] {};

	size_t offset = 1;
	bool response_detected = false;
	gps_abstime time_started = hrt_absolute_time();
	send_message("\n\r");

	// Read buffer to get the COM port
	do {
		--offset; //overwrite the null-char
		int ret = read(reinterpret_cast<uint8_t *>(buf) + offset, sizeof(buf) - offset - 1, SBF_CONFIG_TIMEOUT);

		if (ret < 0) {
			// something went wrong when polling or reading
			SBF_WARN("sbf poll_or_read err");
			return ret;

		}

		offset += ret;
		buf[offset++] = '\0';


		char *p = strstr(buf, ">");

		if (p) { //check if the length of the com port == 4 and contains a > sign
			for (int i = 0; i < 4; i++) {
				com_port[i] = buf[i];
			}

			response_detected = true;
		}

		if (offset >= sizeof(buf)) {
			offset = 1;
		}

	} while (time_started + 1000 * SBF_CONFIG_TIMEOUT > hrt_absolute_time() && !response_detected);

	if (response_detected) {
		SBF_INFO("Septentrio GNSS receiver COM port: %s", com_port);
		response_detected = false; // for future use

	} else {
		SBF_WARN("No COM port detected")
		return -1;
	}

	// Delete all sbf outputs on current COM port to remove clutter data
	char msg[MSG_SIZE];
	snprintf(msg, sizeof(msg), SBF_CONFIG_RESET, com_port);

	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		return -1; // connection and/or baudrate detection failed
	}

	// Set baut rate
	snprintf(msg, sizeof(msg), SBF_CONFIG_BAUDRATE, com_port, baudrate);

	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		SBF_DEBUG("Connection and/or baudrate detection failed (SBF_CONFIG_BAUDRATE)");
		return -1; // connection and/or baudrate detection failed
	}

	// Flush input and wait for at least 50 ms silence
	decodeInit();
	receive(50);
	decodeInit();


	// At this point we have correct baudrate on both ends
	SBF_DEBUG("Correct baud rate on both ends");

	// Define/inquire the type of data that the receiver should accept/send on a given connection descriptor
	snprintf(msg, sizeof(msg), SBF_DATA_IO, com_port);

	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		return -1;
	}

	// Specify the offsets that the receiver applies to the computed attitude angles.
	snprintf(msg, sizeof(msg), SBF_CONFIG_ATTITUDE_OFFSET, (double)(_heading_offset * 180 / M_PI_F), (double)_pitch_offset);

	if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
		return -1;
	}

	// TODO: Make this configurable? There was a variable in the original driver, but unused...
	// Set the type of dynamics the GNSS antenna is subjected to.
	snprintf(msg, sizeof(msg), SBF_CONFIG_RECEIVER_DYNAMICS, "high");
	send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT);

	decodeInit();
	receive(50);
	decodeInit();

	// Output a set of SBF blocks on a given connection at a regular interval.
	int i = 0;
	snprintf(msg, sizeof(msg), SBF_CONFIG, com_port);

	do {
		++i;

		if (!send_message_and_wait_for_ack(msg, SBF_CONFIG_TIMEOUT)) {
			if (i >= 5) {
				return -1; // connection and/or baudrate detection failed
			}

		} else {
			response_detected = true;
		}
	} while (i < 5 && !response_detected);

	_configured = true;

	return 0;
}

void SeptentrioGPS::process_until_error()
{
	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;
	int helper_ret;
	unsigned receive_timeout = TIMEOUT_5HZ;

	while ((helper_ret = _helper->receive(receive_timeout)) > 0 && !should_exit()) {

		if (helper_ret & 1) {
			publish();

			last_rate_count++;
		}

		if (_p_report_sat_info && (helper_ret & 2)) {
			publishSatelliteInfo();
		}

		reset_if_scheduled();

		/* measure update rate every 5 seconds */
		if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
			float dt = (float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f;
			_rate = last_rate_count / dt;
			_rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
			_rate_reading = _num_bytes_read / dt;
			last_rate_measurement = hrt_absolute_time();
			last_rate_count = 0;
			_last_rate_rtcm_injection_count = 0;
			_num_bytes_read = 0;
			_helper->storeUpdateRates();
			_helper->resetUpdateRates();
		}

		if (!_healthy) {
			_healthy = true;
		}
	}
}

int SeptentrioGPS::serial_open()
{
	if (_serial_fd < 0) {
		_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_serial_fd < 0) {
			PX4_ERR("failed to open %s err: %d", _port, errno);
			px4_sleep(1);
			return -1;
		}

#ifdef __PX4_LINUX

		if (_interface == GPSHelper::Interface::SPI) {
			int spi_speed = 1000000; // make sure the bus speed is not too high (required on RPi)
			int status_value = ::ioctl(_serial_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

			if (status_value < 0) {
				PX4_ERR("SPI_IOC_WR_MAX_SPEED_HZ failed for %s (%d)", _port, errno);
			}

			status_value = ::ioctl(_serial_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);

			if (status_value < 0) {
				PX4_ERR("SPI_IOC_RD_MAX_SPEED_HZ failed for %s (%d)", _port, errno);
			}
		}

#endif /* __PX4_LINUX */
	}

	return 0;
}

int SeptentrioGPS::serial_close()
{
	int result = 0;

	if (_serial_fd >= 0) {
		result = ::close(_serial_fd);
		_serial_fd = -1;
	}

	return result;
}

int SeptentrioGPS::parse_char(const uint8_t b)
{
	int ret = 0;

	switch (_decode_state) {

	// Expecting Sync1
	case SBF_DECODE_SYNC1:
		if (b == SBF_SYNC1) { // Sync1 found --> expecting Sync2
			SBF_TRACE_PARSER("A");
			payloadRxAdd(b); // add a payload byte
			_decode_state = SBF_DECODE_SYNC2;

		} else if (b == RTCM3_PREAMBLE && _rtcm_parsing) {
			SBF_TRACE_PARSER("RTCM");
			_decode_state = SBF_DECODE_RTCM3;
			_rtcm_parsing->addByte(b);
		}

		break;

	// Expecting Sync2
	case SBF_DECODE_SYNC2:
		if (b == SBF_SYNC2) { // Sync2 found --> expecting CRC
			SBF_TRACE_PARSER("B");
			payloadRxAdd(b); // add a payload byte
			_decode_state = SBF_DECODE_PAYLOAD;

		} else { // Sync1 not followed by Sync2: reset parser
			decodeInit();
		}

		break;

	// Expecting payload
	case SBF_DECODE_PAYLOAD: SBF_TRACE_PARSER(".");

		ret = payloadRxAdd(b); // add a payload byte

		if (ret < 0) {
			// payload not handled, discard message
			ret = 0;
			decodeInit();

		} else if (ret > 0) {
			ret = payloadRxDone(); // finish payload processing
			decodeInit();

		} else {
			// expecting more payload, stay in state SBF_DECODE_PAYLOAD
			ret = 0;

		}

		break;

	case SBF_DECODE_RTCM3:
		if (_rtcm_parsing->addByte(b)) {
			SBF_DEBUG("got RTCM message with length %i", (int) _rtcm_parsing->messageLength());
			gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
			decodeInit();
		}

		break;

	default:
		break;
	}

	return ret;
}

int SeptentrioGPS::payload_rx_add(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = reinterpret_cast<uint8_t *>(&_buf);

	p_buf[_rx_payload_index++] = b;

	if ((_rx_payload_index > 7 && _rx_payload_index >= _buf.length) || _rx_payload_index >= sizeof(_buf)) {
		ret = 1; // payload received completely
	}

	return ret;
}

int SeptentrioGPS::payload_rx_done()
{
	int ret = 0;
#ifndef NO_MKTIME
	struct tm timeinfo;
	time_t epoch;
#endif

	if (_buf.length <= 4 ||
	    _buf.length > _rx_payload_index ||
	    _buf.crc16 != crc16(reinterpret_cast<uint8_t *>(&_buf) + 4, _buf.length - 4)) {
		return 0;
	}

	// handle message
	switch (_buf.msg_id) {
	case SBF_ID_PVTGeodetic: SBF_TRACE_RXMSG("Rx PVTGeodetic");
		_msg_status |= 1;

		if (_buf.payload_pvt_geodetic.mode_type < 1) {
			_gps_position->fix_type = 1;

		} else if (_buf.payload_pvt_geodetic.mode_type == 6) {
			_gps_position->fix_type = 4;

		} else if (_buf.payload_pvt_geodetic.mode_type == 5 || _buf.payload_pvt_geodetic.mode_type == 8) {
			_gps_position->fix_type = 5;

		} else if (_buf.payload_pvt_geodetic.mode_type == 4 || _buf.payload_pvt_geodetic.mode_type == 7) {
			_gps_position->fix_type = 6;

		} else {
			_gps_position->fix_type = 3;
		}

		// Check fix and error code
		_gps_position->vel_ned_valid = _gps_position->fix_type > 1 && _buf.payload_pvt_geodetic.error == 0;

		// Check boundaries and invalidate GPS velocities
		// We're not just checking for the do-not-use value (-2*10^10) but for any value beyond the specified max values
		if (fabsf(_buf.payload_pvt_geodetic.vn) > 600.0f || fabsf(_buf.payload_pvt_geodetic.ve) > 600.0f ||
		    fabsf(_buf.payload_pvt_geodetic.vu) > 600.0f) {
			_gps_position->vel_ned_valid = false;
		}

		// Check boundaries and invalidate position
		// We're not just checking for the do-not-use value (-2*10^10) but for any value beyond the specified max values
		if (fabs(_buf.payload_pvt_geodetic.latitude) > (double)(M_PI_F / 2.0f) ||
		    fabs(_buf.payload_pvt_geodetic.longitude) > (double) M_PI_F ||
		    fabs(_buf.payload_pvt_geodetic.height) > SBF_PVTGEODETIC_DNU ||
		    fabsf(_buf.payload_pvt_geodetic.undulation) > (float) SBF_PVTGEODETIC_DNU) {
			_gps_position->fix_type = 0;
		}

		if (_buf.payload_pvt_geodetic.nr_sv < 255) {  // 255 = do not use value
			_gps_position->satellites_used = _buf.payload_pvt_geodetic.nr_sv;

			if (_satellite_info) {
				// Only fill in the satellite count for now (we could use the ChannelStatus message for the
				// other data, but it's really large: >800B)
				_satellite_info->timestamp = hrt_absolute_time();
				_satellite_info->count = _gps_position->satellites_used;
				ret = 2;
			}

		} else {
			_gps_position->satellites_used = 0;
		}

		_gps_position->latitude_deg = _buf.payload_pvt_geodetic.latitude * M_RAD_TO_DEG;
		_gps_position->longitude_deg = _buf.payload_pvt_geodetic.longitude * M_RAD_TO_DEG;
		_gps_position->altitude_ellipsoid_m = _buf.payload_pvt_geodetic.height;
		_gps_position->altitude_msl_m = _buf.payload_pvt_geodetic.height - static_cast<double>
						(_buf.payload_pvt_geodetic.undulation);

		/* H and V accuracy are reported in 2DRMS, but based off the uBlox reporting we expect RMS.
		 * Devide by 100 from cm to m and in addition divide by 2 to get RMS. */
		_gps_position->eph = static_cast<float>(_buf.payload_pvt_geodetic.h_accuracy) / 200.0f;
		_gps_position->epv = static_cast<float>(_buf.payload_pvt_geodetic.v_accuracy) / 200.0f;

		_gps_position->vel_n_m_s = static_cast<float>(_buf.payload_pvt_geodetic.vn);
		_gps_position->vel_e_m_s = static_cast<float>(_buf.payload_pvt_geodetic.ve);
		_gps_position->vel_d_m_s = -1.0f * static_cast<float>(_buf.payload_pvt_geodetic.vu);
		_gps_position->vel_m_s = sqrtf(_gps_position->vel_n_m_s * _gps_position->vel_n_m_s +
					       _gps_position->vel_e_m_s * _gps_position->vel_e_m_s);

		_gps_position->cog_rad = static_cast<float>(_buf.payload_pvt_geodetic.cog) * M_DEG_TO_RAD_F;
		_gps_position->c_variance_rad = 1.0f * M_DEG_TO_RAD_F;

		// _buf.payload_pvt_geodetic.cog is set to -2*10^10 for velocities below 0.1m/s
		if (_buf.payload_pvt_geodetic.cog > 360.0f) {
			_buf.payload_pvt_geodetic.cog = NAN;
		}

		_gps_position->time_utc_usec = 0;
#ifndef NO_MKTIME
		/* convert to unix timestamp */
		memset(&timeinfo, 0, sizeof(timeinfo));

		timeinfo.tm_year = 1980 - 1900;
		timeinfo.tm_mon = 0;
		timeinfo.tm_mday = 6 + _buf.WNc * 7;
		timeinfo.tm_hour = 0;
		timeinfo.tm_min = 0;
		timeinfo.tm_sec = _buf.TOW / 1000;

		epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS) {
			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.

			timespec ts;
			memset(&ts, 0, sizeof(ts));
			ts.tv_sec = epoch;
			ts.tv_nsec = (_buf.TOW % 1000) * 1000 * 1000;
			setClock(ts);

			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position->time_utc_usec += (_buf.TOW % 1000) * 1000;
		}

#endif
		_gps_position->timestamp = hrt_absolute_time();
		_last_timestamp_time = _gps_position->timestamp;
		_rate_count_vel++;
		_rate_count_lat_lon++;
		ret |= (_msg_status == 7) ? 1 : 0;
		//SBF_DEBUG("PVTGeodetic handled");
		break;

	case SBF_ID_VelCovGeodetic: SBF_TRACE_RXMSG("Rx VelCovGeodetic");
		_msg_status |= 2;
		_gps_position->s_variance_m_s = _buf.payload_vel_col_geodetic.cov_ve_ve;

		if (_gps_position->s_variance_m_s < _buf.payload_vel_col_geodetic.cov_vn_vn) {
			_gps_position->s_variance_m_s = _buf.payload_vel_col_geodetic.cov_vn_vn;
		}

		if (_gps_position->s_variance_m_s < _buf.payload_vel_col_geodetic.cov_vu_vu) {
			_gps_position->s_variance_m_s = _buf.payload_vel_col_geodetic.cov_vu_vu;
		}

		//SBF_DEBUG("VelCovGeodetic handled");
		break;

	case SBF_ID_DOP: SBF_TRACE_RXMSG("Rx DOP");
		_msg_status |= 4;
		_gps_position->hdop = _buf.payload_dop.hDOP * 0.01f;
		_gps_position->vdop = _buf.payload_dop.vDOP * 0.01f;
		//SBF_DEBUG("DOP handled");
		break;

	case SBF_ID_AttEuler: SBF_TRACE_RXMSG("Rx AttEuler");

		if (!_buf.payload_att_euler.error_not_requested) {

			int error_aux1 = _buf.payload_att_euler.error_aux1;
			int error_aux2 = _buf.payload_att_euler.error_aux2;

			// SBF_DEBUG("Mode: %u", _buf.payload_att_euler.mode)
			if (error_aux1 == 0 && error_aux2 == 0) {
				float heading = _buf.payload_att_euler.heading;
				heading *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]


				if (heading > M_PI_F) {
					heading -= 2.f * M_PI_F; // final range is [-pi, pi]
				}

				_gps_position->heading = heading;
				// SBF_DEBUG("Heading: %.3f rad", (double) _gps_position->heading)
				//SBF_DEBUG("AttEuler handled");

			} else if (error_aux1 != 0) {
				//SBF_DEBUG("Error code for Main-Aux1 baseline: Not enough measurements");
			} else if (error_aux2 != 0) {
				//SBF_DEBUG("Error code for Main-Aux2 baseline: Not enough measurements");
			}
		} else {
			//SBF_DEBUG("AttEuler: attitude not requested by user");
		}


		break;

	case SBF_ID_AttCovEuler: SBF_TRACE_RXMSG("Rx AttCovEuler");

		if (!_buf.payload_att_cov_euler.error_not_requested) {
			int error_aux1 = _buf.payload_att_cov_euler.error_aux1;
			int error_aux2 = _buf.payload_att_cov_euler.error_aux2;

			if (error_aux1 == 0 && error_aux2 == 0) {
				float heading_acc = _buf.payload_att_cov_euler.cov_headhead;
				heading_acc *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]
				_gps_position->heading_accuracy = heading_acc;
				// SBF_DEBUG("Heading-Accuracy: %.3f rad", (double) _gps_position->heading_accuracy)
				//SBF_DEBUG("AttCovEuler handled");

			} else if (error_aux1 != 0) {
				//SBF_DEBUG("Error code for Main-Aux1 baseline: %u: Not enough measurements", error_aux1);
			} else if (error_aux2 != 0) {
				//SBF_DEBUG("Error code for Main-Aux2 baseline: %u: Not enough measurements", error_aux2);
			}
		} else {
			//SBF_DEBUG("AttCovEuler: attitude not requested by user");
		}

		break;

	default:
		break;
	}

	if (ret > 0) {
		_gps_position->timestamp_time_relative = static_cast<int32_t>(_last_timestamp_time - _gps_position->timestamp);
	}

	if (ret == 1) {
		_msg_status &= ~1;
	}

	return ret;
}

void SeptentrioGPS::decode_init()
{
	_decode_state = SBF_DECODE_SYNC1;
	_rx_payload_index = 0;

	if (_output_mode == OutputMode::GPSAndRTCM) {
		if (!_rtcm_parsing) {
			_rtcm_parsing = new RTCMParsing();
		}

		if (_rtcm_parsing) {
			_rtcm_parsing->reset();
		}
	}
}

bool SeptentrioGPS::send_message(const char *msg)
{
	SBF_DEBUG("Send MSG: %s", msg);
	int length = strlen(msg);

	return (write(msg, length) == length);
}

bool SeptentrioGPS::send_message_and_wait_for_ack(const char *msg, const int timeout)
{
	SBF_DEBUG("Send MSG: %s", msg);

	int length = strlen(msg);

	if (write(msg, length) != length) {
		return false;
	}

	// Wait for acknowledge
	// For all valid set -, get - and exe -commands, the first line of the reply is an exact copy
	// of the command as entered by the user, preceded with "$R:"
	char buf[GPS_READ_BUFFER_SIZE];
	size_t offset = 1;
	gps_abstime time_started = hrt_absolute_time();

	bool found_response = false;

	do {
		--offset; //overwrite the null-char
		int ret = read(reinterpret_cast<uint8_t *>(buf) + offset, sizeof(buf) - offset - 1, timeout);

		if (ret < 0) {
			// something went wrong when polling or reading
			SBF_WARN("sbf poll_or_read err");
			return false;
		}

		offset += ret;
		buf[offset++] = '\0';

		if (!found_response && strstr(buf, "$R: ") != nullptr) {
			//SBF_DEBUG("READ %d: %s", (int) offset, buf);
			found_response = true;
		}

		if (offset >= sizeof(buf)) {
			offset = 1;
		}

	} while (time_started + 1000 * timeout > hrt_absolute_time());

	SBF_DEBUG("response: %u", found_response)
	return found_response;
}

int SeptentrioGPS::receive(unsigned timeout)
{
	int ret = 0;
	int handled = 0;
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	if (!_configured) {
		px4_usleep(timeout * 1000);
		return 0;
	}

	// timeout additional to poll
	gps_abstime time_started = hrt_absolute_time();

	while (true) {
		// Wait for only SBF_PACKET_TIMEOUT if something already received.
		ret = read(buf, sizeof(buf), handled ? SBF_PACKET_TIMEOUT : timeout);

		if (ret < 0) {
			// something went wrong when polling or reading
			SBF_WARN("ubx poll_or_read err");
			return -1;

		} else {
			SBF_DEBUG("Read %d bytes (receive)", ret);

			// pass received bytes to the packet decoder
			for (int i = 0; i < ret; i++) {
				handled |= parseChar(buf[i]);
				SBF_DEBUG("parsed %d: 0x%x", i, buf[i]);
			}
		}

		if (handled > 0) {
			return handled;
		}

		// abort after timeout if no useful packets received
		if (time_started + timeout * 1000 < hrt_absolute_time()) {
			SBF_DEBUG("timed out, returning");
			return -1;
		}
	}
}

int SeptentrioGPS::write(const uint8_t* buf, size_t buf_length)
{
	dump_gps_data(buf, buf_length, SeptentrioDumpCommMode::Full, true);

	return ::write(_serial_fd, buf, buf_length);
}

void GPS::dump_gps_data(uint8_t *data, size_t len, SeptentrioDumpCommMode mode, bool msg_to_gps_device)
{
	gps_dump_s *dump_data  = msg_to_gps_device ? _dump_to_device : _dump_from_device;

	if (_dump_communication_mode != mode || !dump_data) {
		return;
	}

	dump_data->instance = (uint8_t)_instance;

	while (len > 0) {
		size_t write_len = len;

		if (write_len > sizeof(dump_data->data) - dump_data->len) {
			write_len = sizeof(dump_data->data) - dump_data->len;
		}

		memcpy(dump_data->data + dump_data->len, data, write_len);
		data += write_len;
		dump_data->len += write_len;
		len -= write_len;

		if (dump_data->len >= sizeof(dump_data->data)) {
			if (msg_to_gps_device) {
				dump_data->len |= 1 << 7;
			}

			dump_data->timestamp = hrt_absolute_time();
			_dump_communication_pub.publish(*dump_data);
			dump_data->len = 0;
		}
	}
}

void SeptentrioGPS::initialize_communication_dump()
{
	param_t handle = param_find("SEPT_DUMP_COMM");
	int32_t param_dump_comm;

	if (handle == PARAM_INVALID || param_get(handle, &param_dump_comm) != 0) {
		return;
	}

	// Check whether dumping is disabled.
	if (param_dump_comm < 1 || param_dump_comm > 2) {
		return;
	}

	_dump_from_device = new gps_dump_s();
	_dump_to_device = new gps_dump_s();

	if (!_dump_from_device || !_dump_to_device) {
		PX4_ERR("failed to allocated dump data");
		return;
	}

	memset(_dump_to_device, 0, sizeof(gps_dump_s));
	memset(_dump_from_device, 0, sizeof(gps_dump_s));

	//make sure to use a large enough queue size, so that we don't lose messages. You may also want
	//to increase the logger rate for that.
	_dump_communication_pub.advertise();

	_dump_communication_mode = (SeptentrioDumpCommMode)param_dump_comm;
}

void SeptentrioGPS::reset_if_scheduled()
{
	SeptentrioGPSResetType reset_type = _scheduled_reset.load();

	if (reset_type != SeptentrioGPSResetType::None) {
		_scheduled_reset.store(GPSRestartType::None);
		int res = reset(reset_type);

		if (res == -1) {
			PX4_INFO("Reset is not supported on this device.");
		} else if (res < 0) {
			PX4_INFO("Reset failed.");
		} else {
			PX4_INFO("Reset succeeded.");
		}
	}
}

// You got this!
extern "C" __EXPORT int septentrio_main(int argc, char *argv[])
{
	return SeptentrioGPS::main(argc, argv);
}
