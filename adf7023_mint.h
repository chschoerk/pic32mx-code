/*
THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES INC. ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT, ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES INC. BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

YOU ASSUME ANY AND ALL RISK FROM THE USE OF THIS CODE OR SUPPORT FILE.

IT IS THE RESPONSIBILITY OF THE PERSON INTEGRATING THIS CODE INTO AN APPLICATION
TO ENSURE THAT THE RESULTING APPLICATION PERFORMS AS REQUIRED AND IS SAFE.

    Module       : adf7023.h
    Description  : ADF7023 Interface
    Date         : 22 June 2010
    Version      : v1.01
    Changelog    : v1.01 radio_cfg_10_afc_scheme_fixed_value should be 2
                         Added ADF_SetDiscrimBW
                         Added ADF_SetDiscrimPhase
    Changelog    : v1.00 Initial
*/

#ifndef _ADF7023_MINT_H_
#define _ADF7023_MINT_H_




#define PKT_RAM_BASE_PTR                    0x10

// Packet RAM Memory
#define MMemMap_Adr_Mask                    0x7FF
#define PR_var_tx_mode_ADR                  0x00D
#define PR_var_params                       1
#define PR_var_tx_mode_ADR                  0x00D
#define BBRam_MMemmap_Start                 0x100
#define MCR_MMemmap_Start                   0x300

// PrF Table 35
#define PARAM_TX_NORMAL_PACKET              0
#define PARAM_TX_PREAMBLE_FOREVER           2
#define PARAM_TX_CARRIER_FOREVER            3

#define LEN_OFFSET              0x0
#define ADR_MATCH_OFFSET        0x1
#define PKT_MAX_PAYLOAD_LEN     4
#define PKT_HDR_LEN             0
#define PKT_MAX_PKT_LEN         (PKT_HDR_LEN + PKT_MAX_PAYLOAD_LEN)


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// ADF7023 Status register - Definitions
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//////////////////////// Firmware States (fw_state) /////////////////////////////////

// The firmware state variable
typedef enum
{
    FW_INIT      = 0x0F, // Initializing
    FW_BUSY      = 0x00, // Busy. Performing a state transition
    FW_OFF       = 0x11,
    FW_ON        = 0x12,
    FW_RX        = 0x13,
    FW_TX        = 0x14,
    FW_RSSI      = 0x05, // Performing CMD_GET_RSSI
    FW_SLEEP     = 0x06, // PHY_SLEEP
    FW_IR_CAL    = 0x07, // Performing CMD_IR_CAL
    FW_AES_INIT  = 0x08, // Performing CMD_AES_INIT
    FW_AES_DEC   = 0x09, // Performing CMD_AES_DEC
    FW_AES_ENC   = 0x0A  // Performing CMD_AES_ENC
} ADF_FwState;

typedef struct
{
   UINT32 fw_state          : 5;   // Indicate the current state of the MAC Processor
   UINT32 cmd_loader_empty  : 1;   // 0:RC not ready for MAC, 1 RC is ready for MAC cmd
   UINT32 irq_status        : 1;   // 1: Pending interrupt condition
   UINT32 spi_ready         : 1;   // 0:SPI is not ready for access, 1 SPI is ready
} ADF_STA;


typedef union
{
   UINT32          Reg;
   ADF_STA       Bits;
} ADFSTA_Reg;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

typedef struct
{                       // Description
                        //
   UINT32 AdrH     :  3;   // 11 bit address
   UINT32 Cmd       : 5;   // 5 bit command
   UINT32 AdrL     :  8;   // 11 bit address
} ADF_MEMCMD;

typedef union
{
   unsigned char ucBytes[0x4];
   UINT32          Reg;
   ADF_MEMCMD    Bits;
} ADFSTA_MemCmd;

/*************************************************************************/
/* CMD Codes                                                             */
/*************************************************************************/
typedef enum
{
   CMD_SYNC                        = 0xA2, // Synchronizes the communication processor to the host microprocessor after rest
   CMD_PHY_OFF                     = 0xB0, // Invoke transition of device into state PHY_OFF
   CMD_PHY_ON                      = 0xB1, // Invoke transition of device into state PHY_ON
   CMD_PHY_RX                      = 0xB2, // Invoke transition of device into state PHY_RX
   CMD_PHY_TX                      = 0xB5, // Invoke transition of device into state PHY_TX
   CMD_PHY_SLEEP                   = 0xBA, // Invoke transition of device into state PHY_SLEEP
   CMD_CONFIG_DEV                  = 0xBB, // Configures the radio parameters based on the BBRAM values.
   CMD_GET_RSSI                    = 0xBC, // Performs an RSSI measurement
   CMD_BB_CAL                      = 0xBE, // Performs an calibration of the IF filter
   CMD_HW_RESET                    = 0xC8, // Performs a full hardware reset. The device enters PHY_SLEEP
   CMD_RAM_LOAD_INIT               = 0xBF, // Prepares the program RAM for a download
   CMD_RAM_LOAD_DONE               = 0xC7, // Performs a reset of the communications processor after loading RAM
   CMD_IR_CAL                      = 0xBD, // Initiates an image rejection calibration using the IR cal code stored on program RAM
   CMD_AES_ENCRYPT                 = 0xD0, // Requires the AES software module
   CMD_AES_DECRYPT_INIT            = 0xD1, // Requires the AES software module
   CMD_AES_DECRYPT                 = 0xD2, // Requires the AES software module
   SPI_MEM_WR                      = 0x18, // Sequential Write
   SPI_MEM_RD                      = 0x38, // Sequential Read
   SPI_MEMR_WR                     = 0x08, // Random Write
   SPI_MEMR_RD                     = 0x28, // Random Read
   SPI_NOP                         = 0xFF  // No operation
} ADF_CmdCodes;


#define KEYTYPE16 12
#define KEYTYPE24 20
#define KEYTYPE32 28

#define ECB 0x0
#define CBC 0x1

#define VAR_C_PTR          16
#define VAR_W_PTR          17
#define VAR_Winv_PTR       18
#define VAR_Wfor_PTR       19
#define VAR_KEYTYPE        20
#define VAR_OPERATION      21
#define VAR_BLOCKMODE      22
#define VAR_ECV_PTR        23
#define VAR_DCV_PTR        24
#define VAR_CIPHERBUF_PTR  25

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// BBRAM Registers
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define BB_interrupt_mask_0_Adr                    0x100
#define BB_interrupt_mask_1_Adr                    0x101
#define BB_number_of_wakeups_0_Adr                 0x102
#define BB_number_of_wakeups_1_Adr                 0x103
#define BB_number_of_wakeups_irq_threshold_0_Adr   0x104
#define BB_number_of_wakeups_irq_threshold_1_Adr   0x105
#define BB_rx_dwell_time_Adr                       0x106
#define BB_parmtime_divider_Adr                    0x107
#define BB_swm_rssi_thresh_Adr                     0x108
#define BB_channel_freq_0_Adr                      0x109
#define BB_channel_freq_1_Adr                      0x10A
#define BB_channel_freq_2_Adr                      0x10B
#define BB_radio_cfg_0_Adr                         0x10C
#define BB_radio_cfg_1_Adr                         0x10D
#define BB_radio_cfg_2_Adr                         0x10E
#define BB_radio_cfg_3_Adr                         0x10F
#define BB_radio_cfg_4_Adr                         0x110
#define BB_radio_cfg_5_Adr                         0x111
#define BB_radio_cfg_6_Adr                         0x112
#define BB_radio_cfg_7_Adr                         0x113
#define BB_radio_cfg_8_Adr                         0x114
#define BB_radio_cfg_9_Adr                         0x115
#define BB_radio_cfg_10_Adr                        0x116
#define BB_radio_cfg_11_Adr                        0x117
#define BB_image_reject_cal_phase_Adr              0x118
#define BB_image_reject_cal_amplitude_Adr          0x119
#define BB_mode_control_Adr                        0x11A
#define BB_preamble_match_Adr                      0x11B
#define BB_symbol_mode_Adr                         0x11C
#define BB_preamble_len_Adr                        0x11D
#define BB_crc_poly_0_Adr                          0x11E
#define BB_crc_poly_1_Adr                          0x11F
#define BB_sync_control_Adr                        0x120
#define BB_sync_byte_0_Adr                         0x121
#define BB_sync_byte_1_Adr                         0x122
#define BB_sync_byte_2_Adr                         0x123
#define BB_tx_base_adr_Adr                         0x124
#define BB_rx_base_adr_Adr                         0x125
#define BB_packet_length_control_Adr               0x126
#define BB_packet_length_max_Adr                   0x127
#define BB_static_reg_fix_Adr                      0x128
#define BB_address_match_offset_Adr                0x129
#define BB_address_length_Adr                      0x12A
#define BB_address_match_byte_0_Adr                0x12B
#define BB_address_mask_byte_0_Adr                 0x12C
#define BB_address_match_byte_1_Adr                0x12D
#define BB_address_mask_byte_1_Adr                 0x12E
#define BB_address_match_byte_2_Adr                0x12F
#define BB_address_mask_byte_2_Adr                 0x130
#define BB_address_match_byte_3_Adr                0x131
#define BB_address_mask_byte_3_Adr                 0x132
#define BB_address_match_byte_4_Adr                0x133
#define BB_address_mask_byte_4_Adr                 0x134
#define BB_address_match_byte_5_Adr                0x135
#define BB_address_mask_byte_5_Adr                 0x136
#define BB_address_match_byte_6_Adr                0x137
#define BB_address_mask_byte_6_Adr                 0x138
#define BB_address_match_byte_7_Adr                0x139
#define BB_address_mask_byte_7_Adr                 0x13A
#define BB_address_match_byte_8_Adr                0x13B
#define BB_address_mask_byte_8_Adr                 0x13C
#define BB_rx_synth_lock_time_Adr               0x13E
#define BB_tx_synth_lock_time_Adr               0x13F

typedef struct
{
    unsigned char interrupt_mask_0_r;                   // 0x100
    unsigned char interrupt_mask_1_r;                   // 0x101
    unsigned char number_of_wakeups_0_r;                // 0x102
    unsigned char number_of_wakeups_1_r;                // 0x103
    unsigned char number_of_wakeups_irq_threshold_0_r;  // 0x104
    unsigned char number_of_wakeups_irq_threshold_1_r;  // 0x105
    unsigned char rx_dwell_time_r;                      // 0x106
    unsigned char parmtime_divider_r;                   // 0x107
    unsigned char swm_rssi_thresh_r;                    // 0x108
    unsigned char channel_freq_0_r;                     // 0x109
    unsigned char channel_freq_1_r;                     // 0x10A
    unsigned char channel_freq_2_r;                     // 0x10B
    unsigned char radio_cfg_0_r;                        // 0x10C
    unsigned char radio_cfg_1_r;                        // 0x10D
    unsigned char radio_cfg_2_r;                        // 0x10E
    unsigned char radio_cfg_3_r;                        // 0x10F
    unsigned char radio_cfg_4_r;                        // 0x110
    unsigned char radio_cfg_5_r;                        // 0x111
    unsigned char radio_cfg_6_r;                        // 0x112
    unsigned char radio_cfg_7_r;                        // 0x113
    unsigned char radio_cfg_8_r;                        // 0x114
    unsigned char radio_cfg_9_r;                        // 0x115
    unsigned char radio_cfg_10_r;                       // 0x116
    unsigned char radio_cfg_11_r;                       // 0x117
    unsigned char image_reject_cal_phase_r;             // 0x118
    unsigned char image_reject_cal_amplitude_r;         // 0x119
    unsigned char mode_control_r;                       // 0x11A
    unsigned char preamble_match_r;                     // 0x11B
    unsigned char symbol_mode_r;                        // 0x11C
    unsigned char preamble_len_r;                       // 0x11D
    unsigned char crc_poly_0_r;                         // 0x11E
    unsigned char crc_poly_1_r;                         // 0x11F
    unsigned char sync_control_r;                       // 0x120
    unsigned char sync_byte_0_r;                        // 0x121
    unsigned char sync_byte_1_r;                        // 0x122
    unsigned char sync_byte_2_r;                        // 0x123
    unsigned char tx_base_adr_r;                        // 0x124
    unsigned char rx_base_adr_r;                        // 0x125
    unsigned char packet_length_control_r;              // 0x126
    unsigned char packet_length_max_r;                  // 0x127
    unsigned char static_reg_fix_r;                     // 0x128
    unsigned char address_match_offset_r;               // 0x129
    //unsigned char address_filtering_r[0x14];            // 0x12A - 0x13D
    unsigned char address_length_r;                       // 0x12A
    unsigned char address_filtering_r[13];                // 0x12B-0x137
    unsigned char rssi_wait_time_r;                       // 0x138
    unsigned char testmodes_r;                            // 0x139
    unsigned char transition_clock_div_r;                 // 0x13A
    unsigned char placeholder_dummy_r[3];                   // 0x13B-0x13D
    unsigned char rx_synth_lock_time_r;                 // 0x13E
    unsigned char tx_synth_lock_time_r;                 // 0x13F
} TyBBRAM;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// MCR Registers
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define MCR_pa_level_mcr_Adr                            0x307
#define MCR_wuc_config_high_Adr                         0x30C
#define MCR_wuc_config_low_Adr                          0x30D
#define MCR_wuc_value_high_Adr                          0x30E
#define MCR_wuc_value_low_Adr                           0x30F

#define MCR_wuc_flag_reset_Adr                          0x310
#define MCR_wuc_status_Adr                              0x311
#define MCR_rssi_readback_Adr                           0x312
#define MCR_max_afc_range_Adr                           0x315
#define MCR_image_reject_cal_config_Adr                 0x319

#define MCR_chip_shutdown_Adr                           0x322
#define MCR_powerdown_aux_Adr                           0x325
#define MCR_adc_readback_high_Adr                       0x327
#define MCR_adc_readback_low_Adr                        0x328
#define MCR_silicon_rev0_Adr                            0x329
#define MCR_silicon_rev1_Adr                            0x32a
#define MCR_silicon_rev2_Adr                            0x32b
#define MCR_silicon_rev3_Adr                            0x32c
#define MCR_battery_monitor_threshold_voltage_Adr       0x32d
#define MCR_ext_uc_clk_divide_Adr                       0x32e
#define MCR_agc_clk_divide_Adr                          0x32f

#define MCR_interrupt_source_0_Adr                      0x336
#define MCR_interrupt_source_1_Adr                      0x337
#define MCR_calibration_control_Adr                     0x338
#define MCR_calibration_status_Adr                      0x339
#define MCR_image_reject_calibration_status_Adr         0x33a

#define MCR_rxbb_cal_calwrd_readback_Adr                0x345
#define MCR_rxbb_cal_calwrd_overwrite_Adr               0x346

#define MCR_adc_config_low_Adr                          0x359
#define MCR_adc_config_high_Adr                         0x35a
#define MCR_agc_ook_control_Adr                         0x35b
#define MCR_agc_config_Adr                              0x35c
#define MCR_agc_mode_Adr                                0x35d
#define MCR_agc_low_threshold_Adr                       0x35e
#define MCR_agc_high_threshold_Adr                      0x35f

#define MCR_agc_gain_status_Adr                         0x360

#define MCR_frequency_error_readback_Adr                0x372

#define MCR_vco_band_ovrw_val_Adr                       0x3cb
#define MCR_vco_ampl_ovrw_val_Adr                       0x3cc
#define MCR_vco_ovrw_en_Adr                             0x3cd

#define MCR_vco_cal_cfg_Adr                             0x3d0
#define MCR_osc_and_doubler_config_Adr                  0x3d2
#define MCR_vco_band_readback_Adr                       0x3da
#define MCR_vco_ampl_readback_Adr                       0x3db

#define MCR_analog_test_bus_six_Adr                     0x3f8
#define MCR_rssi_tstmux_sel_Adr                         0x3f9
#define MCR_gpio_configure_Adr                          0x3fa
#define MCR_test_dac_gain_Adr                           0x3fd


// MCR_gpio_configure_Adr
#define gpio_configure_default         0x00
#define gpio_configure_cdr_slicer_mode 0x21
#define gpio_configure_sport_mode_0    0xA0
#define gpio_configure_sport_mode_1    0xA1
#define gpio_configure_sport_mode_2    0xA2
#define gpio_configure_sport_mode_3    0xA3
#define gpio_configure_sport_mode_4    0xA4
#define gpio_configure_sport_mode_5    0xA5
#define gpio_configure_sport_mode_6    0xA6
#define gpio_configure_sport_mode_7    0xA7
#define gpio_configure_sport_mode_8    0xA8


typedef struct
{
   unsigned char r_300;                                 // 0x300
   unsigned char r_301;                                 // 0x301
   unsigned char r_302;                                 // 0x302
   unsigned char r_303;                                 // 0x303
   unsigned char r_304;                                 // 0x304
   unsigned char r_305;                                 // 0x305
   unsigned char r_306;                                 // 0x306
   unsigned char pa_level_mcr_r;                        // 0x307
   unsigned char r_308;                                 // 0x308
   unsigned char r_309;                                 // 0x309
   unsigned char r_30a;                                 // 0x30a
   unsigned char r_30b;                                 // 0x30b
   unsigned char wuc_config_high_r;                     // 0x30c
   unsigned char wuc_config_low_r;                      // 0x30d
   unsigned char wuc_value_high_r;                      // 0x30e
   unsigned char wuc_value_low_r;                       // 0x30f
                                                        //
   unsigned char wuc_flag_reset_r;                      // 0x310
   unsigned char wuc_status_r;                          // 0x311
   unsigned char rssi_readback_r;                       // 0x312
   unsigned char r_313;                                 // 0x313
   unsigned char r_314;                                 // 0x314
   unsigned char max_afc_range_r;                       // 0x315
   unsigned char r_316;                                 // 0x316
   unsigned char r_317;                                 // 0x317
   unsigned char r_318;                                 // 0x318
   unsigned char image_reject_cal_config_r;             // 0x319
   unsigned char r_31a;                                 // 0x31a
   unsigned char r_31b;                                 // 0x31b
   unsigned char r_31c;                                 // 0x31c
   unsigned char r_31d;                                 // 0x31d
   unsigned char r_31e;                                 // 0x31e
   unsigned char r_31f;                                 // 0x31f
                                                        //
   unsigned char r_320;                                 // 0x320
   unsigned char r_321;                                 // 0x321
   unsigned char chip_shutdown_r;                       // 0x322
   unsigned char r_323;                                 // 0x323
   unsigned char r_324;                                 // 0x324
   unsigned char powerdown_aux_r;                       // 0x325
   unsigned char r_326;                                 // 0x326
   unsigned char adc_readback_high_r;                   // 0x327
   unsigned char adc_readback_low_r;                    // 0x328
   unsigned char silicon_rev0_r;                        // 0x329
   unsigned char silicon_rev1_r;                        // 0x32a
   unsigned char silicon_rev2_r;                        // 0x32b
   unsigned char silicon_rev3_r;                        // 0x32c
   unsigned char battery_monitor_threshold_voltage_r;   // 0x32d
   unsigned char ext_uc_clk_divide_r;                   // 0x32e
   unsigned char agc_clk_divide_r;                      // 0x32f
                                                        //
   unsigned char r_330;                                 // 0x330
   unsigned char r_331;                                 // 0x331
   unsigned char r_332;                                 // 0x332
   unsigned char r_333;                                 // 0x333
   unsigned char r_334;                                 // 0x334
   unsigned char r_335;                                 // 0x335
   unsigned char interrupt_source_0_r;                  // 0x336
   unsigned char interrupt_source_1_r;                  // 0x337
   unsigned char calibration_control_r;                 // 0x338
   unsigned char calibration_status_r;                  // 0x339
   unsigned char image_reject_calibration_status_r;     // 0x33a
   unsigned char r_33b;                                 // 0x33b
   unsigned char r_33c;                                 // 0x33c
   unsigned char r_33d;                                 // 0x33d
   unsigned char r_33e;                                 // 0x33e
   unsigned char r_33f;                                 // 0x33f
                                                        //
   unsigned char r_340;                                 // 0x340
   unsigned char r_341;                                 // 0x341
   unsigned char r_342;                                 // 0x342
   unsigned char r_343;                                 // 0x343
   unsigned char r_344;                                 // 0x344
   unsigned char rxbb_cal_calwrd_readback_r;            // 0x345
   unsigned char rxbb_cal_calwrd_overwrite_r;           // 0x346
   unsigned char r_347;                                 // 0x347
   unsigned char r_348;                                 // 0x348
   unsigned char r_349;                                 // 0x349
   unsigned char r_34a;                                 // 0x34a
   unsigned char r_34b;                                 // 0x34b
   unsigned char r_34c;                                 // 0x34c
   unsigned char r_34d;                                 // 0x34d
   unsigned char r_34e;                                 // 0x34e
   unsigned char r_34f;                                 // 0x34f
                                                        //
   unsigned char r_350;                                 // 0x350
   unsigned char r_351;                                 // 0x351
   unsigned char r_352;                                 // 0x352
   unsigned char r_353;                                 // 0x353
   unsigned char r_354;                                 // 0x354
   unsigned char r_355;                                 // 0x355
   unsigned char r_356;                                 // 0x356
   unsigned char r_357;                                 // 0x357
   unsigned char r_358;                                 // 0x358
   unsigned char adc_config_low_r;                      // 0x359
   unsigned char adc_config_high_r;                     // 0x35a
   unsigned char agc_ook_control_r;                     // 0x35b
   unsigned char agc_config_r;                          // 0x35c
   unsigned char agc_mode_r;                            // 0x35d
   unsigned char agc_low_threshold_r;                   // 0x35e
   unsigned char agc_high_threshold_r;                  // 0x35f
                                                        //
   unsigned char agc_gain_status_r;                     // 0x360
   unsigned char r_361;                                 // 0x361
   unsigned char r_362;                                 // 0x362
   unsigned char r_363;                                 // 0x363
   unsigned char r_364;                                 // 0x364
   unsigned char r_365;                                 // 0x365
   unsigned char r_366;                                 // 0x366
   unsigned char r_367;                                 // 0x367
   unsigned char r_368;                                 // 0x368
   unsigned char r_369;                                 // 0x369
   unsigned char r_36A;                                 // 0x36A
   unsigned char r_36b;                                 // 0x36b
   unsigned char r_36c;                                 // 0x36c
   unsigned char r_36d;                                 // 0x36d
   unsigned char r_36e;                                 // 0x36e
   unsigned char r_36f;                                 // 0x36f
                                                        //
   unsigned char r_370;                                 // 0x370
   unsigned char r_371;                                 // 0x371
   unsigned char frequency_error_readback_r;            // 0x372
   unsigned char r_373;                                 // 0x373
   unsigned char r_374;                                 // 0x374
   unsigned char r_375;                                 // 0x375
   unsigned char r_376;                                 // 0x376
   unsigned char r_377;                                 // 0x377
   unsigned char r_378;                                 // 0x378
   unsigned char r_379;                                 // 0x379
   unsigned char r_37A;                                 // 0x37A
   unsigned char r_37b;                                 // 0x37b
   unsigned char r_37c;                                 // 0x37c
   unsigned char r_37d;                                 // 0x37d
   unsigned char r_37e;                                 // 0x37e
   unsigned char r_37f;                                 // 0x37f
                                                        //
   unsigned char r_380;                                 // 0x380
   unsigned char r_381;                                 // 0x381
   unsigned char r_382;                                 // 0x382
   unsigned char r_383;                                 // 0x383
   unsigned char r_384;                                 // 0x384
   unsigned char r_385;                                 // 0x385
   unsigned char r_386;                                 // 0x386
   unsigned char r_387;                                 // 0x387
   unsigned char r_388;                                 // 0x388
   unsigned char r_389;                                 // 0x389
   unsigned char r_38A;                                 // 0x38A
   unsigned char r_38b;                                 // 0x38b
   unsigned char r_38c;                                 // 0x38c
   unsigned char r_38d;                                 // 0x38d
   unsigned char r_38e;                                 // 0x38e
   unsigned char r_38f;                                 // 0x38f
                                                        //
   unsigned char r_390;                                 // 0x390
   unsigned char r_391;                                 // 0x391
   unsigned char r_392;                                 // 0x392
   unsigned char r_393;                                 // 0x393
   unsigned char r_394;                                 // 0x394
   unsigned char r_395;                                 // 0x395
   unsigned char r_396;                                 // 0x396
   unsigned char r_397;                                 // 0x397
   unsigned char r_398;                                 // 0x398
   unsigned char r_399;                                 // 0x399
   unsigned char r_39a;                                 // 0x39a
   unsigned char r_39b;                                 // 0x39b
   unsigned char r_39c;                                 // 0x39c
   unsigned char r_39d;                                 // 0x39d
   unsigned char r_39e;                                 // 0x39e
   unsigned char r_39f;                                 // 0x39f
                                                        //
   unsigned char r_3a0;                                 // 0x3a0
   unsigned char r_3a1;                                 // 0x3a1
   unsigned char r_3a2;                                 // 0x3a2
   unsigned char r_3a3;                                 // 0x3a3
   unsigned char r_3a4;                                 // 0x3a4
   unsigned char r_3a5;                                 // 0x3a5
   unsigned char r_3a6;                                 // 0x3a6
   unsigned char r_3a7;                                 // 0x3a7
   unsigned char r_3a8;                                 // 0x3a8
   unsigned char r_3a9;                                 // 0x3a9
   unsigned char r_3aa;                                 // 0x3aa
   unsigned char r_3ab;                                 // 0x3ab
   unsigned char r_3ac;                                 // 0x3ac
   unsigned char r_3ad;                                 // 0x3ad
   unsigned char r_3ae;                                 // 0x3ae
   unsigned char r_3af;                                 // 0x3af
                                                        //
   unsigned char r_3b0;                                 // 0x3b0
   unsigned char r_3b1;                                 // 0x3b1
   unsigned char r_3b2;                                 // 0x3b2
   unsigned char r_3b3;                                 // 0x3b3
   unsigned char r_3b4;                                 // 0x3b4
   unsigned char r_3b5;                                 // 0x3b5
   unsigned char r_3b6;                                 // 0x3b6
   unsigned char r_3b7;                                 // 0x3b7
   unsigned char r_3b8;                                 // 0x3b8
   unsigned char r_3b9;                                 // 0x3b9
   unsigned char r_3ba;                                 // 0x3ba
   unsigned char r_3bb;                                 // 0x3bb
   unsigned char r_3bc;                                 // 0x3bc
   unsigned char r_3bd;                                 // 0x3bd
   unsigned char r_3be;                                 // 0x3be
   unsigned char r_3bf;                                 // 0x3bf
                                                        //
   unsigned char r_3c0;                                 // 0x3c0
   unsigned char r_3c1;                                 // 0x3c1
   unsigned char r_3c2;                                 // 0x3c2
   unsigned char r_3c3;                                 // 0x3c3
   unsigned char r_3c4;                                 // 0x3c4
   unsigned char r_3c5;                                 // 0x3c5
   unsigned char r_3c6;                                 // 0x3c6
   unsigned char r_3c7;                                 // 0x3c7
   unsigned char r_3c8;                                 // 0x3c8
   unsigned char r_3c9;                                 // 0x3c9
   unsigned char r_3ca;                                 // 0x3ca
   unsigned char vco_band_ovrw_val_r;                   // 0x3cb
   unsigned char vco_ampl_ovrw_val_r;                   // 0x3cc
   unsigned char vco_ovrw_en_r;                         // 0x3cd
   unsigned char r_3ce;                                 // 0x3ce
   unsigned char r_3cf;                                 // 0x3cf
                                                        //
   unsigned char vco_cal_cfg_r;                         // 0x3d0
   unsigned char r_3d1;                                 // 0x3d1
   unsigned char osc_and_doubler_config_r;              // 0x3d2
   unsigned char r_3d3;                                 // 0x3d3
   unsigned char r_3d4;                                 // 0x3d4
   unsigned char r_3d5;                                 // 0x3d5
   unsigned char r_3d6;                                 // 0x3d6
   unsigned char r_3d7;                                 // 0x3d7
   unsigned char r_3d8;                                 // 0x3d8
   unsigned char r_3d9;                                 // 0x3d9
   unsigned char vco_band_readback_r;                   // 0x3da
   unsigned char vco_ampl_readback_r;                   // 0x3db
   unsigned char r_3dc;                                 // 0x3dc
   unsigned char r_3dd;                                 // 0x3dd
   unsigned char r_3de;                                 // 0x3de
   unsigned char r_3df;                                 // 0x3df
                                                        //
   unsigned char r_3e0;                                 // 0x3e0
   unsigned char r_3e1;                                 // 0x3e1
   unsigned char r_3e2;                                 // 0x3e2
   unsigned char r_3e3;                                 // 0x3e3
   unsigned char r_3e4;                                 // 0x3e4
   unsigned char r_3e5;                                 // 0x3e5
   unsigned char r_3e6;                                 // 0x3e6
   unsigned char r_3e7;                                 // 0x3e7
   unsigned char r_3e8;                                 // 0x3e8
   unsigned char r_3e9;                                 // 0x3e9
   unsigned char r_3ea;                                 // 0x3ea
   unsigned char r_3eb;                                 // 0x3eb
   unsigned char r_3ec;                                 // 0x3ec
   unsigned char r_3ed;                                 // 0x3ed
   unsigned char r_3ee;                                 // 0x3ee
   unsigned char r_3ef;                                 // 0x3ef
                                                        //
   unsigned char r_3f0;                                 // 0x3f0
   unsigned char r_3f1;                                 // 0x3f1
   unsigned char r_3f2;                                 // 0x3f2
   unsigned char r_3f3;                                 // 0x3f3
   unsigned char r_3f4;                                 // 0x3f4
   unsigned char r_3f5;                                 // 0x3f5
   unsigned char r_3f6;                                 // 0x3f6
   unsigned char r_3f7;                                 // 0x3f7
   unsigned char analog_test_bus_six_r;                 // 0x3f8
   unsigned char rssi_tstmux_sel_r;                     // 0x3f9
   unsigned char gpio_configure_r;                      // 0x3fa
   unsigned char r_3fb;                                 // 0x3fb
   unsigned char r_3fc;                                 // 0x3fc
   unsigned char test_dac_gain_r;                       // 0x3fd
   unsigned char r_3fe;                                 // 0x3fe
   unsigned char r_3ff;                                 // 0x3ff
} TyMCR;


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
unsigned char   ADF_MMapRead            (unsigned long ulAdr,unsigned long ulLen,unsigned char *pData);
unsigned char   ADF_MMapWrite           (unsigned long ulAdr,unsigned long ulLen,unsigned char *pData);
BOOL         ADF_TransmitCarrier     (void);
BOOL         ADF_TransmitPreamble    (void);
BOOL         ADF_TransmitPacket      (UINT8 * pucTXData,UINT8 ucLen);
BOOL         ADF_ReceivePacket       (void);
BOOL         ADF_FirstConnect        (void);
BOOL         ADF_GoToOnState         (void);
ADF_FwState     ADF_GetFwState          (void);
void            ADF_CheckInt            (void);
void            ADF_PrintBBRAM          (TyBBRAM *pBBRAM);
BOOL         ADF_BBRAMReadBack       (void);
BOOL         ADF_ConfigureRadio      (TyBBRAM *pBBRAM);
void            ADF_SetChannelFreq      (TyBBRAM *pBBRAM,unsigned long ulChannelFreq);
void            ADF_SetFreqDev          (TyBBRAM *pBBRAM,unsigned long ulFreqDev);
void            ADF_SetDataRate         (TyBBRAM *pBBRAM,unsigned long ulDataRate);
void            ADF_SetPALevel          (TyBBRAM *pBBRAM,unsigned long ulPALevel);
void            ADF_SetDiscrimBW        (TyBBRAM *pBBRAM,unsigned long ulDiscrimBW);
void            ADF_SetDiscrimPhase     (TyBBRAM *pBBRAM,unsigned long ulDiscrimPhase);
BOOL         ADF_AES                 (void);
BOOL         ADF_WaitForCmdComplete  (void);

BOOL        ADF_SyncComms       (void);
void        ADF_BBRAMDefault    (TyBBRAM *pBBRAM);
void        ADF_XMit            (unsigned char ucByte,unsigned char *pData);
BOOL        ADF_IssueCommandNW  (unsigned char Cmd);
BOOL        ADF_IssueCommand    (unsigned char Cmd);
BOOL        ADF_ReadStatus      (ADFSTA_Reg *pStatus);
BOOL        ADF_WaitCmdLdr      (void);
ADF_FwState ADF_GetFwState      (void);
BOOL        ADF_WaitFWState     (ADF_FwState FWState);

int         setupADF(void);
BOOL        ADF_Init            (void);
BOOL        ADF_waitForMISOToGoHigh();
BOOL        ADF_MCRRegisterReadBack(TyMCR *pMCRin);
BOOL        ADF_GoToRxState(void);
BOOL        ADF_GoToTxState(void);
void        ADF_XMit_softwareSPI(unsigned char ucByte,unsigned char *pData);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// BBRAM Constants
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#define interrupt_mask_0_interrupt_num_wakeups           (0x1 << 7)
#define interrupt_mask_0_interrupt_swm_rssi_det          (0x1 << 6)
#define interrupt_mask_0_interrupt_aes_done              (0x1 << 5)
#define interrupt_mask_0_interrupt_tx_eof                (0x1 << 4)
#define interrupt_mask_0_interrupt_address_match         (0x1 << 3)
#define interrupt_mask_0_interrupt_crc_correct           (0x1 << 2)
#define interrupt_mask_0_interrupt_sync_detect           (0x1 << 1)
#define interrupt_mask_0_interrupt_premable_detect       (0x1 << 0)

#define interrupt_mask_1_battery_alarm                   (0x1 << 7)
#define interrupt_mask_1_cmd_ready                       (0x1 << 6)
#define interrupt_mask_1_wuc_timeout                     (0x1 << 4)
#define interrupt_mask_1_spi_ready                       (0x1 << 1)
#define interrupt_mask_1_cmd_finished                    (0x1 << 0)

#define packet_length_control_length_offset_numbits      (3)
#define packet_length_control_length_offset_offset       (0)
#define packet_length_control_length_offset_minus4       (0x0 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_minus3       (0x1 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_minus2       (0x2 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_minus1       (0x3 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_minus0       (0x4 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_plus1        (0x5 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_plus2        (0x6 << packet_length_control_length_offset_offset)
#define packet_length_control_length_offset_plus3        (0x7 << packet_length_control_length_offset_offset)

#define packet_length_control_data_mode_numbits          (2)
#define packet_length_control_data_mode_offset           (3)
#define packet_length_control_data_mode_packet           (0x0 << packet_length_control_data_mode_offset)
#define packet_length_control_data_mode_sport_preamble   (0x1 << packet_length_control_data_mode_offset)
#define packet_length_control_data_mode_sport_sync       (0x2 << packet_length_control_data_mode_offset)
#define packet_length_control_data_mode_unused           (0x3 << packet_length_control_data_mode_offset)

#define packet_length_control_crc_en_no                  (0x0 << 5)
#define packet_length_control_crc_en_yes                 (0x1 << 5)

#define packet_length_control_packet_len_variable        (0x0 << 6)
#define packet_length_control_packet_len_fixed           (0x1 << 6)

#define packet_length_control_data_byte_lsb              (0x0 << 7)
#define packet_length_control_data_byte_msb              (0x1 << 7)

#define sync_control_sync_word_length_numbits            (5)
#define sync_control_sync_word_length_offset             (0)
#define sync_control_sync_word_length_0                  (0  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_1                  (1  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_2                  (2  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_3                  (3  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_4                  (4  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_5                  (5  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_6                  (6  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_7                  (7  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_8                  (8  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_9                  (9  << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_10                 (10 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_11                 (11 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_12                 (12 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_13                 (13 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_14                 (14 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_15                 (15 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_16                 (16 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_17                 (17 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_18                 (18 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_19                 (19 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_20                 (20 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_21                 (21 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_22                 (22 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_23                 (23 << sync_control_sync_word_length_offset)
#define sync_control_sync_word_length_24                 (24 << sync_control_sync_word_length_offset)

#define sync_control_sync_error_tol_numbits              (2)
#define sync_control_sync_error_tol_offset               (6)
#define sync_control_sync_error_tol_0_errors_allowed     (0 << sync_control_sync_error_tol_offset)
#define sync_control_sync_error_tol_1_errors_allowed     (1 << sync_control_sync_error_tol_offset)
#define sync_control_sync_error_tol_2_errors_allowed     (2 << sync_control_sync_error_tol_offset)
#define sync_control_sync_error_tol_3_errors_allowed     (3 << sync_control_sync_error_tol_offset)

#define symbol_mode_symbol_length_8_bit                  (0 << 0)
#define symbol_mode_symbol_length_10_bit                 (1 << 0)

#define symbol_mode_data_whitening_disabled              (0 << 3)
#define symbol_mode_data_whitening_enabled               (1 << 3)

#define symbol_mode_eight_ten_enc_disabled               (0 << 4 )
#define symbol_mode_eight_ten_enc_enabled                (1 << 4 )

#define symbol_mode_prog_crc_en_disabled                 (0 << 5)
#define symbol_mode_prog_crc_en_enabled                  (1 << 5)

#define symbol_mode_manchester_enc_disabled              (0 << 6)
#define symbol_mode_manchester_enc_enabled               (1 << 6)

#define preamble_match_qual_disabled                     (0x0 << 0)
#define preamble_match_4_in_24_win                       (0x8 << 0)
#define preamble_match_3_in_24_win                       (0x9 << 0)
#define preamble_match_2_in_24_win                       (0xA << 0)
#define preamble_match_1_in_24_win                       (0xB << 0)
#define preamble_match_0_in_24_win                       (0xC << 0)

#define mode_control_swm_en_disabled                     (0x0 << 7)
#define mode_control_swm_en_enabled                      (0x1 << 7)

#define mode_control_bb_cal_disabled                     (0x0 << 6)
#define mode_control_bb_cal_enabled                      (0x1 << 6)

#define mode_control_swm_rssi_qual_disabled              (0x0 << 5)
#define mode_control_swm_rssi_qual_enabled               (0x1 << 5)

#define mode_control_tx_auto_turnaround_disabled         (0x0 << 4)
#define mode_control_tx_auto_turnaround_enabled          (0x1 << 4)

#define mode_control_rx_auto_turnaround_disabled         (0x0 << 3)
#define mode_control_rx_auto_turnaround_enabled          (0x1 << 3)

#define mode_control_custom_trx_synth_lock_time_en_disabled (0x0 << 2)
#define mode_control_custom_trx_synth_lock_time_en_enabled  (0x1 << 2)

#define mode_control_ext_lna_en_disabled                 (0x0 << 1)
#define mode_control_ext_lna_en_enabled                  (0x1 << 1)

#define mode_control_ext_pa_en_disabled                  (0x0 << 0)
#define mode_control_ext_pa_en_enabled                   (0x1 << 0)


#define radio_cfg_11_afc_kp_2_numbits                    (4)
#define radio_cfg_11_afc_kp_2_offset                     (4)
#define radio_cfg_11_afc_kp_2_power_0                    (0  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_1                    (1  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_2                    (2  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_3                    (3  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_4                    (4  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_5                    (5  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_6                    (6  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_7                    (7  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_8                    (8  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_9                    (9  << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_10                   (10 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_11                   (11 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_12                   (12 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_13                   (13 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_14                   (14 << radio_cfg_11_afc_kp_2_offset)
#define radio_cfg_11_afc_kp_2_power_15                   (15 << radio_cfg_11_afc_kp_2_offset)

#define radio_cfg_11_afc_ki_2_numbits                    (4)
#define radio_cfg_11_afc_ki_2_offset                     (0)
#define radio_cfg_11_afc_ki_2_power_0                    (0  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_1                    (1  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_2                    (2  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_3                    (3  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_4                    (4  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_5                    (5  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_6                    (6  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_7                    (7  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_8                    (8  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_9                    (9  << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_10                   (10 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_11                   (11 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_12                   (12 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_13                   (13 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_14                   (14 << radio_cfg_11_afc_ki_2_offset)
#define radio_cfg_11_afc_ki_2_power_15                   (15 << radio_cfg_11_afc_ki_2_offset)

#define radio_cfg_10_afc_lock_mode_numbits               (2)
#define radio_cfg_10_afc_lock_mode_offset                (0)
#define radio_cfg_10_afc_lock_mode_free_running          (0x0  << radio_cfg_10_afc_lock_mode_offset)
#define radio_cfg_10_afc_lock_mode_disabled              (0x1  << radio_cfg_10_afc_lock_mode_offset)
#define radio_cfg_10_afc_lock_mode_paused                (0x2  << radio_cfg_10_afc_lock_mode_offset)
#define radio_cfg_10_afc_lock_mode_lock_after_preamble   (0x3  << radio_cfg_10_afc_lock_mode_offset)

#define radio_cfg_10_afc_scheme_numbits                  (2)
#define radio_cfg_10_afc_scheme_offset                   (2)
#define radio_cfg_10_afc_scheme_fixed_value              (0x2  << radio_cfg_10_afc_scheme_offset)

#define radio_cfg_10_afc_polarity_numbits                (1)
#define radio_cfg_10_afc_polarity_offset                 (4)
#define radio_cfg_10_afc_polarity_fixed_value            (0x0  << radio_cfg_10_afc_polarity_offset)

#define radio_cfg_9_demod_scheme_numbits                 (3)
#define radio_cfg_9_demod_scheme_offset                  (0)
#define radio_cfg_9_demod_scheme_FSK                     (0x0  << radio_cfg_9_demod_scheme_offset)
#define radio_cfg_9_demod_scheme_GFSK                    (0x1  << radio_cfg_9_demod_scheme_offset)
#define radio_cfg_9_demod_scheme_OOK                     (0x2  << radio_cfg_9_demod_scheme_offset)

#define radio_cfg_9_mod_scheme_numbits                   (3)
#define radio_cfg_9_mod_scheme_offset                    (3)
#define radio_cfg_9_mod_scheme_2_level_FSK               (0x0  << radio_cfg_9_mod_scheme_offset)
#define radio_cfg_9_mod_scheme_2_level_GFSK              (0x1  << radio_cfg_9_mod_scheme_offset)
#define radio_cfg_9_mod_scheme_OOK                       (0x2  << radio_cfg_9_mod_scheme_offset)
#define radio_cfg_9_mod_scheme_carrier_only              (0x3  << radio_cfg_9_mod_scheme_offset)

#define radio_cfg_9_ifbw_numbits                         (2)
#define radio_cfg_9_ifbw_offset                          (6)
#define radio_cfg_9_ifbw_100kHz                          (0x0  << radio_cfg_9_ifbw_offset)
#define radio_cfg_9_ifbw_150kHz                          (0x1  << radio_cfg_9_ifbw_offset)
#define radio_cfg_9_ifbw_200kHz                          (0x2  << radio_cfg_9_ifbw_offset)
#define radio_cfg_9_ifbw_300kHz                          (0x3  << radio_cfg_9_ifbw_offset)

#define radio_cfg_8_pa_single_diff_sel_single_ended      (0x0  << 7)
#define radio_cfg_8_pa_single_diff_sel_differential      (0x1  << 7)

#define radio_cfg_8_pa_power_numbits                     (4)
#define radio_cfg_8_pa_power_offset                      (3)

#define radio_cfg_8_pa_power_setting_3                   (0x0 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_7                   (0x1 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_11                  (0x2 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_15                  (0x3 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_19                  (0x4 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_23                  (0x5 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_27                  (0x6 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_31                  (0x7 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_35                  (0x8 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_39                  (0x9 << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_43                  (0xA << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_47                  (0xB << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_51                  (0xC << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_55                  (0xD << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_59                  (0xE << radio_cfg_8_pa_power_offset)
#define radio_cfg_8_pa_power_setting_63                  (0xF << radio_cfg_8_pa_power_offset)

#define radio_cfg_8_pa_ramp_numbits                      (3)
#define radio_cfg_8_pa_ramp_offset                       (0)
#define radio_cfg_8_pa_ramp_256                          (0x1 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_128                          (0x2 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_64                           (0x3 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_32                           (0x4 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_16                           (0x5 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_8                            (0x6 << radio_cfg_8_pa_ramp_offset)
#define radio_cfg_8_pa_ramp_4                            (0x7 << radio_cfg_8_pa_ramp_offset)

// AGC lock mode
#define radio_cfg_7_agc_lock_mode_numbits                (2)
#define radio_cfg_7_agc_lock_mode_offset                 (6)
#define radio_cfg_7_agc_lock_mode_free_running           (0x0  << radio_cfg_7_agc_lock_mode_offset)
#define radio_cfg_7_agc_lock_mode_manual                 (0x1  << radio_cfg_7_agc_lock_mode_offset)
#define radio_cfg_7_agc_lock_mode_hold                   (0x2  << radio_cfg_7_agc_lock_mode_offset)
#define radio_cfg_7_agc_lock_mode_lock_after_preamble    (0x3  << radio_cfg_7_agc_lock_mode_offset)

#define radio_cfg_7_synth_lut_control_numbits             (2)
#define radio_cfg_7_synth_lut_control_offset              (4)
#define radio_cfg_7_synth_lut_control_predef_rx_predef_tx (0x0  << radio_cfg_7_synth_lut_control_offset)
#define radio_cfg_7_synth_lut_control_custom_rx_predef_tx (0x1  << radio_cfg_7_synth_lut_control_offset)
#define radio_cfg_7_synth_lut_control_predef_rx_custom_tx (0x2  << radio_cfg_7_synth_lut_control_offset)
#define radio_cfg_7_synth_lut_control_custom_rx_custom_tx (0x3  << radio_cfg_7_synth_lut_control_offset)

#define radio_cfg_7_synth_lut_config_1_numbits            (4)
#define radio_cfg_7_synth_lut_config_1_offset             (0)



#endif // #ifndef _ADF7023_MINT_H_