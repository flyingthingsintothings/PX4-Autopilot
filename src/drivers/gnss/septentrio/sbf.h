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

#pragma once

/**
 * @file sbf.h
 * @brief Septentrio binary format (SBF) protocol definitions.
 */

#define SBF_SYNC1           0x24     ///< Leading '$' of SBF blocks
#define SBF_SYNC2           0x40     ///< Leading '@' of SBF blocks
#define SBF_PVTGEODETIC_DNU 100000.0 ///< Do-Not-Use value for PVTGeodetic

// Block IDs
#define SBF_ID_DOP            4001
#define SBF_ID_PVTGeodetic    4007
#define SBF_ID_ChannelStatus  4013
#define SBF_ID_ReceiverStatus 4014
#define SBF_ID_QualityInd     4082
#define SBF_ID_RFStatus       4092
#define SBF_ID_GalAuthStatus  4245
#define SBF_ID_VelCovGeodetic 5908
#define SBF_ID_AttEuler       5938
#define SBF_ID_AttCovEuler    5939

#pragma pack(push, 1) // SBF protocol binary message and payload definitions

typedef struct {
	uint8_t mode_type: 4;       /**< Bit field indicating the PVT mode type, as follows:
                                     0: No PVT available (the Error field indicates the cause of the absence of the PVT solution)
                                     1: Stand-Alone PVT
                                     2: Differential PVT
                                     3: Fixed location
                                     4: RTK with fixed ambiguities
                                     5: RTK with float ambiguities
                                     6: SBAS aided PVT
                                     7: moving-base RTK with fixed ambiguities
                                     8: moving-base RTK with float ambiguities
                                     10:Precise Point Positioning (PPP) */
	uint8_t mode_reserved: 2;   /**< Reserved */
	uint8_t mode_base_fixed: 1; /**< Set if the user has entered the command setPVTMode,base,auto and the receiver
                                     is still in the process of determining its fixed position. */
	uint8_t mode_2d: 1;        /**< 2D/3D flag: set in 2D mode(height assumed constant and not computed). */
	uint8_t error;             /**< PVT error code. The following values are defined:
                                     0: No Error
                                     1: Not enough measurements
                                     2: Not enough ephemerides available
                                     3: DOP too large (larger than 15)
                                     4: Sum of squared residuals too large
                                     5: No convergence
                                     6: Not enough measurements after outlier rejection
                                     7: Position output prohibited due to export laws
                                     8: Not enough differential corrections available
                                     9: Base station coordinates unavailable
                                     10:Ambiguities not fixed and user requested to only output RTK-fixed positions
                                     Note: if this field has a non-zero value, all following fields are set to their Do-Not-Use value. */
	double latitude;          /**< Marker latitude, from -PI/2 to +PI/2, positive North of Equator */
	double longitude;         /**< Marker longitude, from -PI to +PI, positive East of Greenwich */
	double height;            /**< Marker ellipsoidal height (with respect to the ellipsoid specified by Datum) */
	float undulation;        /**< Geoid undulation computed from the global geoid model defined in
                                     the document 'Technical Characteristics of the NAVSTAR GPS, NATO, June 1991' */
	float vn;                /**< Velocity in the North direction */
	float ve;                /**< Velocity in the East direction */
	float vu;                /**< Velocity in the Up direction */
	float cog;               /**< Course over ground: this is defined as the angle of the vehicle with respect
                                     to the local level North, ranging from 0 to 360, and increasing towards east.
                                     Set to the do-not-use value when the speed is lower than 0.1m/s. */
	double rx_clk_bias;       /**< Receiver clock bias relative to system time reported in the Time System field.
                                     To transfer the receiver time to the system time, use: tGPS/GST=trx-RxClkBias */
	float RxClkDrift;        /**< Receiver clock drift relative to system time (relative frequency error) */
	uint8_t time_system;       /**< Time system of which the offset is provided in this sub-block:
                                     0:GPStime
                                     1:Galileotime
                                     3:GLONASStime */
	uint8_t datum;             /**< This field defines in which datum the coordinates are expressed:
                                     0: WGS84/ITRS
                                     19: Datum equal to that used by the DGNSS/RTK basestation
                                     30: ETRS89(ETRF2000 realization)
                                     31: NAD83(2011), North American Datum(2011)
                                     32: NAD83(PA11), North American Datum, Pacificplate (2011)
                                     33: NAD83(MA11), North American Datum, Marianas plate(2011)
                                     34: GDA94(2010), Geocentric Datum of Australia (2010)
                                     250:First user-defined datum
                                     251:Second user-defined datum */
	uint8_t nr_sv;             /**< Total number of satellites used in the PVT computation. */
	uint8_t wa_corr_info;      /**< Bit field providing information about which wide area corrections have been applied:
                                     Bit 0: set if orbit and satellite clock correction information is used
                                     Bit 1: set if range correction information is used
                                     Bit 2: set if ionospheric information is used
                                     Bit 3: set if orbit accuracy information is used(UERE/SISA)
                                     Bit 4: set if DO229 Precision Approach mode is active
                                     Bits 5-7: Reserved */
	uint16_t reference_id;      /**< In case of DGPS or RTK operation, this field is to be interpreted as the base station identifier.
                                     In SBAS operation, this field is to be interpreted as the PRN of the geostationary satellite
                                     used (from 120 to 158). If multiple base stations or multiple geostationary satellites are used
                                     the value is set to 65534.*/
	uint16_t mean_corr_age;     /**< In case of DGPS or RTK, this field is the mean age of the differential corrections.
                                     In case of SBAS operation, this field is the mean age of the 'fast corrections'
                                     provided by the SBAS satellites */
	uint32_t signal_info;       /**< Bit field indicating the type of GNSS signals having been used in the PVT computations.
                                     If a bit i is set, the signal type having index i has been used. */
	uint8_t alert_flag;         /**< Bit field indicating integrity related information */

	// Revision 1
	uint8_t nr_bases;
	uint16_t ppp_info;
	// Revision 2
	uint16_t latency;
	uint16_t h_accuracy;
	uint16_t v_accuracy;
} sbf_payload_pvt_geodetic_t;

typedef struct {
	uint8_t  cpu_load;                           ///< Load on the receiver’s CPU. The load should stay below 80% in normal
                                                     ///< operation. Higher loads might result in data loss.

	/* Bit field reporting external errors, i.e. errors detected in external data.
           Upon detection of an error, the corresponding bit is set for a duration of
           one second, and then resets. */
	uint8_t  ext_error_siserror: 1;              ///< SISERROR: set if a violation of the signal-in-space ICD has been
                                                     ///< detected for at least one satellite while that satellite is re-
                                                     ///< ported as healthy. Use the command "lif,SisError" for
                                                     ///< details.
	uint8_t  ext_error_diff_corr_error: 1;       ///< DIFFCORRERROR: set when an anomaly has been detected
                                                     ///< in an incoming differential correction stream, causing the re-
                                                     ///< ceiver to fail to decode the corrections. Use the command
                                                     ///< "lif,DiffCorrError" for details.
	uint8_t  ext_error_ext_sensor_error: 1;      ///< EXTSENSORERROR: set when a malfunction has been de-
                                                     ///< tected on at least one of the external sensors connected to
                                                     ///< the receiver. Use the command "lif, ExtSensorError"
                                                     ///< for details.
	uint8_t  ext_error_setup_error: 1;           ///< SETUPERROR: set when a configuration/setup error has been
                                                     ///< detected. An example of such error is when a remote
                                                     ///< NTRIP Caster is not reachable. Use the command "lif,
                                                     ///< SetupError" for details.
	uint8_t  ext_error_reserved: 4;              ///< Reserved

	uint32_t uptime;                             ///< Number of seconds elapsed since the start-up of the receiver, or since
                                                     ///< the last reset.

	/* Bit field indicating the status of key components of the receiver. */
	uint32_t rx_state_reserved1: 1;              ///< Reserved
	uint32_t rx_state_active_antenna: 1;         ///< ACTIVEANTENNA: this bit is set when an active GNSS antenna
                                                     ///< is sensed, i.e. when current is drawn from the antenna con-
                                                     ///< nector.
	uint32_t rx_state_ext_freq: 1;               ///< EXT_FREQ: this bit is set if an external frequency reference is
                                                     ///< detected at the 10 MHz input, and cleared if the receiver uses
                                                     ///< its own internal clock.
	uint32_t rx_state_ext_time: 1;               ///< EXT_TIME: this bit is set if a pulse has been detected on the
                                                     ///< TimeSync input.
	uint32_t rx_state_wn_set: 1;                 ///< WNSET: see corresponding bit in the SyncLevel field of the
                                                     ///< ReceiverTime block.
	uint32_t rx_state_tow_set: 1;                ///< TOWSET: see corresponding bit in the SyncLevel field of the
                                                     ///< ReceiverTime block.
	uint32_t rx_state_fine_time: 1;              ///< FINETIME: see corresponding bit in the SyncLevel field of
                                                     ///< the ReceiverTime block.
	uint32_t rx_state_internal_disk_activity: 1; ///< INTERNALDISK_ACTIVITY: this bit is set for one second each
                                                     ///< time data is logged to the internal disk (DSK1). If the logging
                                                     ///< rate is larger than 1 Hz, set continuously.
	uint32_t rx_state_internal_disk_full: 1;     ///< INTERNALDISK_FULL: this bit is set when the internal disk
                                                     ///< (DSK1) is full. A disk is full when it is filled to 95% of its total
                                                     ///< capacity.
	uint32_t rx_state_internal_disk_mounted: 1;  ///< INTERNALDISK_MOUNTED: this bit is set when the internal
                                                     ///< disk (DSK1) is mounted.
	uint32_t rx_state_int_ant: 1;                ///< INT_ANT: this bit is set when the GNSS RF signal is taken from
                                                     ///< the internal antenna input, and cleared when it comes from
                                                     ///< the external antenna input (only applicable on receiver mod-
                                                     ///< els featuring an internal antenna input).
	uint32_t rx_state_refout_locked: 1;          ///< REFOUT_LOCKED: if set, the 10-MHz frequency provided at
                                                     ///< the REF OUT connector is locked to GNSS time. Otherwise it
                                                     ///< is free-running.
	uint32_t rx_state_reserved2: 1;              ///< Reserved
	uint32_t rx_state_external_disk_activity: 1; ///< EXTERNALDISK_ACTIVITY: this bit is set for one second each
                                                     ///< time data is logged to the external disk (DSK2). If the logging
                                                     ///< rate is larger than 1 Hz, set continuously.
	uint32_t rx_state_external_disk_full: 1;     ///< EXTERNALDISK_FULL: this bit is set when the external disk
                                                     ///< (DSK2) is full. A disk is full when it is filled to 95% of its total
                                                     ///< capacity.
	uint32_t rx_state_external_disk_mounted: 1;  ///< EXTERNALDISK_MOUNTED: this bit is set when the external
                                                     ///< disk (DSK2) is mounted.
	uint32_t rx_state_pps_in_cal: 1;             ///< PPS_IN_CAL: this bit is set when PPS IN delay calibration is on-
                                                     ///< going. Only applicable to PolaRx5TR receivers.
	uint32_t rx_state_diff_corr_in: 1;           ///< DIFFCORR_IN: this bit is set for one second each time differ-
                                                     ///< ential corrections are decoded. If the input rate is larger than
                                                     ///< 1 Hz, set continuously.
	uint32_t rx_state_internet: 1;               ///< INTERNET: this bit is set when the receiver has Internet access.
                                                     ///< If not set, there is either no Internet access, or the receiver
                                                     ///< could not reliably determine the status.
	uint32_t rx_state_reserved3: 13;             ///< Reserved

	/* Bit field indicating whether an error occurred previously. If this field is
           not equal to zero, at least one error has been detected. */
	uint32_t rx_error_reserved1: 3;              ///< Reserved
	uint32_t rx_error_software: 1;               ///< SOFTWARE: set upon detection of a software warning or error.
                                                     ///< This bit is reset by the command "lif, error".
	uint32_t rx_error_watchdog: 1;               ///< WATCHDOG: set when the watchdog expired at least once
                                                     ///< since the last power-on.
	uint32_t rx_error_antenna: 1;                ///< ANTENNA: set when antenna overcurrent condition is de-
                                                     ///< tected.
	uint32_t rx_error_congestion: 1;             ///< CONGESTION: set when an output data congestion has been
                                                     ///< detected on at least one of the communication ports of the
                                                     ///< receiver during the last second.
	uint32_t rx_error_reserved2: 1;              ///< Reserved
	uint32_t rx_error_missed_event: 1;           ///< MISSEDEVENT: set when an external event congestion has
                                                     ///< been detected during the last second. It indicates that the re-
                                                     ///< ceiver is receiving too many events on its EVENTx pins.
	uint32_t rx_error_cpu_overload: 1;           ///< CPUOVERLOAD: set when the CPU load is larger than 90%.
	uint32_t rx_error_invalid_config: 1;         ///< INVALIDCONFIG: set if one or more configuration file (e.g. per-
                                                     ///< missions) is invalid or absent.
	uint32_t rx_error_out_of_geofence: 1;        ///< OUTOFGEOFENCE: set if the receiver is currently out of its per-
                                                     ///< mitted region of operation (geofencing).
	uint32_t rx_error_reserved3: 22;             ///< Reserved

	uint8_t n;                                   ///< Number of AGCState sub-blocks this block contains.
	uint8_t sb_length;                           ///< Length of a AGCState sub-block.
	uint8_t cmd_count;                           ///< Command cyclic counter, incremented each time a command is entered
                                                     ///< that changes the receiver configuration. After the counter has reached
                                                     ///< 255, it resets to 1.
	uint8_t temperature;                         ///< Receiver temperature with an offset of 100. Remove 100 to get the tem-
                                                     ///< perature in degree Celsius.
} sbf_payload_receiver_status_t;

typedef struct {
	uint8_t type;        ///< Quality indicator type:
                             ///<  0:  Overall quality
                             ///<  1:  GNSS signals from main antenna
                             ///<  2:  GNSS signals from aux1 antenna
                             ///<  11: RF power level from the main antenna
                             ///<  12: RF power level from the aux1 antenna
                             ///<  21: CPU headroom
                             ///<  25: OCXO stability (only available on PolaRx5S re-
                             ///<      ceivers)
                             ///<  30: Base station measurements. This indicator is
                             ///<      only available in RTK mode. A low value could
                             ///<      for example hint at severe multipath or inter-
                             ///<      ference at the base station, or also at iono-
                             ///<      spheric scintillation.
                             ///<  31: RTK post-processing. This indicator is only
                             ///<      available when the position mode is not RTK.
                             ///<      It indicates the likelihood of getting a cm-
                             ///<      accurate RTK position when post-processing
                             ///<      the current data.
	uint8_t value: 4;    ///< Value of this quality indicator (from 0 for low quality
                             ///< to 10 for high quality, or 15 if unknown)
	uint8_t reserved: 4; ///< Reserved for future use, to be ignored by decoding
                             ///< software.
} sbf_payload_quality_ind_sub_t;

typedef struct {
	uint8_t n;            ///< Number of quality indicators contained in this block
	uint8_t reserved;     ///< Reserved for future use, to be ignored by decoding software.
	sbf_payload_quality_ind_sub_t *indicators;
} sbf_payload_quality_ind_t;

typedef struct {
	uint32_t frequency;          ///< Center frequency of the RF band addressed by this sub-block.
	uint16_t bandwidth;          ///< Bandwidth of the RF band.

	uint8_t  info_mode: 4;       ///< Mode:
                                     ///<   1: This RF band is suppressed by a notch filter set manually with the com-
                                     ///<      mand setNotchFiltering.
                                     ///<   2: The receiver detected interference in this band, and successfully can-
                                     ///<      celed it.
                                     ///<   8: The receiver detected interference in this band. No mitigation applied.
	uint8_t  info_reserved: 2;   ///< Reserved
	uint8_t  info_antenna_id: 2; ///< 0 for main, 1 for Aux1 and 2 for Aux2
} sbf_payload_rf_status_rf_band_t;

typedef struct {
	uint8_t n;                                       ///< Number of RF bands for which data is provided in this SBF block, i.e.
                                                         ///< number of RFBand sub-blocks.
	uint8_t sb_length;                               ///< Length of one sub-block

	/* Bit field "Flags" */
	uint8_t flags_inauthentic_gnss_signals: 1;       ///< Set when the receiver determined that the GNSS signals at its RF
                                                         ///< connector may not be authentic and that its output (position or
                                                         ///< raw measurements) may therefore be misleading. The receiver
                                                         ///< may be connected to a GNSS simulator, or be subject to a spoof-
                                                         ///< ing attack.
                                                         ///< This bit is based on a set of built-in tests to check the authen-
                                                         ///< ticity of the GNSS signals. In addition to those tests, Navigation
                                                         ///< Message Authentication (NMA) is performed as well. If NMA ver-
                                                         ///< ification fails, bit 1 is set instead.
                                                         ///< Note that bit 0 may be set even if no interference is detected (i.e.
                                                         ///< with no associated RFBand sub-blocks).
	uint8_t flags_inauthentic_navigation_message: 1; ///< Set when a non-authentic navigation message is detected by
                                                         ///< NMA checks (e.g. Galileo OSNMA or Fugro AtomiChron NMA).
	uint8_t flags_reserved: 6;                       ///< Reserved

	uint8_t reserved[3];                             ///< Reserved for future use, to be ignored by decoding software.
	uint8_t rf_band;                                 ///< A succession of N RFBand sub-blocks, see definition below
	uint8_t blocks;                                  ///< Start byte of the sub-blocks.
} sbf_payload_rf_status_t;

typedef struct {
	/* Bit field "OSNMAStatus" */
	uint16_t osnma_status_status: 3;                  ///< status:
                                                          ///<   0: Disabled
                                                          ///<   1: Initializing
                                                          ///<   2: Waiting for trusted time information
                                                          ///<   3: Init failed - inconsistent time
                                                          ///<   4: Init failed - KROOT signature invalid
                                                          ///<   5: Init failed - invalid param received
                                                          ///<   6: Authenticating
	uint16_t osnma_status_initialization_progress: 8; ///< OSNMA initialization progress, expressed in percent [0-100].
                                                          ///< This value will only be encoded when the OSNMA Status is
                                                          ///< initializing. A value of 255 indicates an alert condition of the
                                                          ///< OSNMA operation resulting in OSNMA not being available.
	uint16_t osnma_status_trusted_time_source: 3;     ///< Trusted time source:
                                                          ///<   0: NTP
                                                          ///<   1: L-Band
                                                          ///<   7: Unknown
	uint16_t osnma_status_merkle_tree_busy: 1;        ///< Indicates if the Merkle tree renewal is in progress:
                                                          ///<   0: No
                                                          ///<   1: Yes
	uint16_t osnma_status_reserved: 1;                ///< Reserved

	float trusted_time_delta;                         ///< Time difference between external trusted and receiver time, positive
                                                          ///< when receiver time lags trusted time.
	uint64_t gal_active_mask;                         ///< Bit field indicating the Galileo satellites for which OSNMA results are
                                                          ///< available. If bit i is set, OSNMA authentication is available for Galileo
                                                          ///< satellite i+1.
	uint64_t gal_authentic_mask;                      ///< Bit field indicating the Galileo satellites successfully authenticated by
                                                          ///< OSNMA. If bit i is set, the navigation message from Galileo satellite
                                                          ///< i+1 is authentic. If bit i is not set and the corresponding bit is set in
                                                          ///< GalActiveMask, the navigation message from that satellite is non-
                                                          ///< authentic.
	uint64_t gps_active_mask;                         ///< Bit field indicating the GPS satellites for which OSNMA results are avail-
                                                          ///< able. If bit i is set, OSNMA authentication is available for GPS satellite
                                                          ///< i+1.
	uint64_t gps_authentic_mask;                      ///< Bit field indicating the GPS satellites successfully authenticated by OS-
                                                          ///< NMA. If bit i is set, the navigation message from GPS satellite i+1
                                                          ///< is authentic. If bit i is not set and the corresponding bit is set in
                                                          ///< GpsActiveMask, the navigation message from that satellite is non-
                                                          ///< authentic.
} sbf_payload_gal_auth_status_t;

typedef struct {
	uint8_t mode_type: 4;       /**< Bit field indicating the PVT mode type, as follows:
                                     0: No PVT available (the Error field indicates the cause of the absence of the PVT solution)
                                     1: Stand-Alone PVT
                                     2: Differential PVT
                                     3: Fixed location
                                     4: RTK with fixed ambiguities
                                     5: RTK with float ambiguities
                                     6: SBAS aided PVT
                                     7: moving-base RTK with fixed ambiguities
                                     8: moving-base RTK with float ambiguities
                                     10:Precise Point Positioning (PPP) */
	uint8_t mode_reserved: 2;  /**< Reserved */
	uint8_t mode_base_fixed: 1;/**< Set if the user has entered the command setPVTMode,base,auto and the receiver
                                     is still in the process of determining its fixed position. */
	uint8_t mode_2d: 1;        /**< 2D/3D flag: set in 2D mode(height assumed constant and not computed). */
	uint8_t error;             /**< PVT error code. The following values are defined:
                                     0: No Error
                                     1: Not enough measurements
                                     2: Not enough ephemerides available
                                     3: DOP too large (larger than 15)
                                     4: Sum of squared residuals too large
                                     5: No convergence
                                     6: Not enough measurements after outlier rejection
                                     7: Position output prohibited due to export laws
                                     8: Not enough differential corrections available
                                     9: Base station coordinates unavailable
                                     10:Ambiguities not fixed and user requested to only output RTK-fixed positions
                                     Note: if this field has a non-zero value, all following fields are set to their Do-Not-Use value. */
	float cov_vn_vn;            /**< Variance of the north-velocity estimate */
	float cov_ve_ve;            /**< Variance of the east-velocity estimate */
	float cov_vu_vu;            /**< Variance of the up - velocity estimate */
	float cov_dt_dt;            /**< Variance of the clock drift estimate */
	float cov_vn_ve;            /**< Covariance between the north - and east - velocity estimates */
	float cov_vn_vu;            /**< Covariance between the north - and up - velocity estimates */
	float cov_vn_dt;            /**< Covariance between the north - velocity and clock drift estimates */
	float cov_ve_vu;            /**< Covariance between the east - and up - velocity estimates */
	float cov_ve_dt;            /**< Covariance between the east - velocity and clock drift estimates */
	float cov_vu_dt;            /**< Covariance between the up - velocity and clock drift estimates */
} sbf_payload_vel_cov_geodetic_t;

typedef struct {
	uint8_t nr_sv;              /**< Total number of satellites used in the PVT computation. */
	uint8_t reserved;
	uint16_t pDOP;
	uint16_t tDOP;
	uint16_t hDOP;
	uint16_t vDOP;
	float hpl;                  /**< Horizontal Protection Level (see the DO229 standard). */
	float vpl;                  /**< Vertical Protection Level (see the DO229 standard). */
} sbf_payload_dop_t;

typedef struct {
	uint8_t antenna;
	uint8_t reserved;
	uint16_t tracking_status;
	uint16_t pvt_status;
	uint16_t pvt_info;
} sbf_payload_channel_state_info_t;

typedef struct {
	uint8_t nr_sv;                  /**< The average over all antennas of the number of satellites currently included in the attitude calculations. */
	uint8_t error_aux1: 2;          /**< Bits 0-1: Error code for Main-Aux1 baseline:
                                            0: No error
                                            1: Not enough measurements
                                            2: Reserved
                                            3: Reserved */
	uint8_t error_aux2: 2;          /**< Bits 2-3: Error code for Main-Aux2 baseline, same definition as bit 0-1. */
	uint8_t error_reserved: 3;      /**< Bits 4-6: Reserved */
uint8_t error_not_requested:
	1; /**< Bit 7: Set when GNSS-based attitude not requested by user. In that case, the other bits are all zero. */

	uint16_t mode;                  /**< Attitude mode code:
                                            0: No attitude
                                            1: Heading, pitch (roll = 0), aux antenna positions obtained with float
                                            ambiguities
                                            2: Heading, pitch (roll = 0), aux antenna positions obtained with fixed
                                            ambiguities
                                            3: Heading, pitch, roll, aux antenna positions obtained with float ambiguities
                                            4: Heading, pitch, roll, aux antenna positions obtained with fixed ambiguities */
	uint16_t reserved;              /**< Reserved for future use, to be ignored by decoding software */

	float heading;                  /**< Heading */
	float pitch;                    /**< Pitch */
	float roll;                     /**< Roll */
	float pitch_dot;                /**< Rate of change of the pitch angle */
	float roll_dot;                 /**< Rate of change of the roll angle */
	float heading_dot;              /**< Rate of change of the heading angle */
} sbf_payload_att_euler;

typedef struct {
	uint8_t reserved;               /**< Reserved for future use, to be ignored by decoding software */

	uint8_t error_aux1: 2;          /**< Bits 0-1: Error code for Main-Aux1 baseline:
                                            0: No error
                                            1: Not enough measurements
                                            2: Reserved
                                            3: Reserved */
	uint8_t error_aux2: 2;          /**< Bits 2-3: Error code for Main-Aux2 baseline, same definition as bit 0-1. */
	uint8_t error_reserved: 3;      /**< Bits 4-6: Reserved */
uint8_t error_not_requested:
	1; /**< Bit 7: Set when GNSS-based attitude not requested by user. In that case, the other bits are all zero. */

	float cov_headhead;             /**< Variance of the heading estimate */
	float cov_pitchpitch;           /**< Variance of the pitch estimate */
	float cov_rollroll;             /**< Variance of the roll estimate */
	float cov_headpitch;            /**< Covariance between Euler angle estimates.
                                         Future functionality. The values are currently set to their Do-Not-Use values. */
	float cov_headroll;             /**< Covariance between Euler angle estimates.
                                         Future functionality. The values are currently set to their Do-Not-Use values. */
	float cov_pitchroll;            /**< Covariance between Euler angle estimates.
                                         Future functionality. The values are currently set to their Do-Not-Use values. */
} sbf_payload_att_cov_euler;

// General message and payload buffer union

typedef struct {
	uint16_t sync;              /** The Sync field is a 2-byte array always set to 0x24, 0x40. The first byte of every SBF block has
                                        hexadecimal value 24 (decimal 36, ASCII '$'). The second byte of every SBF block has hexadecimal
                                        value 40 (decimal 64, ASCII '@'). */
	uint16_t crc16;             /** The CRC field is the 16-bit CRC of all the bytes in an SBF block from and including the ID field
                                        to the last byte of the block. The generator polynomial for this CRC is the so-called CRC-CCITT
                                        polynomial: x 16 + x 12 + x 5 + x 0 . The CRC is computed in the forward direction using a seed of 0, no
                                        reverse and no final XOR. */
uint16_t msg_id:
	13;                 /** The ID field is a 2-byte block ID, which uniquely identifies the block type and its contents */
uint8_t msg_revision:
	3;                  /** block revision number, starting from 0 at the initial block definition, and incrementing
                                        each time backwards - compatible changes are performed to the block  */
	uint16_t length;            /** The Length field is a 2-byte unsigned integer containing the size of the SBF block.
                                        It is the total number of bytes in the SBF block including the header.
                                        It is always a multiple of 4. */
	uint32_t TOW;               /** Time-Of-Week: Time-tag, expressed in whole milliseconds from
                                        the beginning of the current Galileo/GPSweek. */
	uint16_t WNc;               /** The GPS week number associated with the TOW. WNc is a continuous
                                        weekcount (hence the "c"). It is not affected by GPS week roll overs,
                                        which occur every 1024 weeks. By definition of the Galileo system time,
                                        WNc is also the Galileo week number + 1024. */
	union {
		sbf_payload_pvt_geodetic_t payload_pvt_geodetic;
		sbf_payload_receiver_status_t payload_receiver_status;
		sbf_payload_quality_ind_t payload_quality_ind;
		sbf_payload_rf_status_t payload_rf_status;
		sbf_payload_gal_auth_status_t payload_gal_auth_status;
		sbf_payload_vel_cov_geodetic_t payload_vel_col_geodetic;
		sbf_payload_dop_t payload_dop;
		sbf_payload_att_euler payload_att_euler;
		sbf_payload_att_cov_euler payload_att_cov_euler;
	};

	uint8_t padding[16];
} sbf_buf_t;

#pragma pack(pop) // End of SBF protocol binary message and payload definitions
