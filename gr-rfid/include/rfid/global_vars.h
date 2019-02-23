/* -*- c++ -*- */
/*
 * Copyright 2015 <Nikos Kargas (nkargas@isc.tuc.gr)>.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_RFID_GLOBAL_VARS_H
#define INCLUDED_RFID_GLOBAL_VARS_H

#include <rfid/api.h>
#include <map>
#include <sys/time.h>
#include <fstream>

//#define DEBUG_MESSAGE

namespace gr {
  namespace rfid {

    enum STATUS               {RUNNING, TERMINATED};
    enum GEN2_LOGIC_STATUS  {SEND_QUERY, SEND_ACK, SEND_QUERY_REP, IDLE, SEND_CW, START, SEND_QUERY_ADJUST, SEND_NAK_QR, SEND_NAK_Q, POWER_DOWN};
    enum GATE_STATUS        {GATE_OPEN, GATE_CLOSED, GATE_SEEK_RN16, GATE_SEEK_EPC};
    enum DECODER_STATUS     {DECODER_DECODE_RN16, DECODER_DECODE_EPC};

    struct READER_STATS
    {
      int n_queries_sent;
      int n_ack_sent;

      int cur_inventory_round;
      int cur_slot_number;

      int max_slot_number;
      int max_inventory_round;

      int n_epc_correct;

      std::vector<std::string> ack_sent;
      std::map<int,int> tag_reads;

      struct timeval start, end;
    };

    struct READER_STATE
    {
      STATUS               status;
      GEN2_LOGIC_STATUS   gen2_logic_status;
      GATE_STATUS         gate_status;
      DECODER_STATUS       decoder_status;
      READER_STATS         reader_stats;



      std::vector<float> magn_squared_samples; // used for sync
      int n_samples_to_ungate; // used by the GATE and DECODER block
    };

    // CONSTANTS (READER CONFIGURATION)

    // Fixed number of slots (2^(FIXED_Q))
    const int FIXED_Q              = 0;

    // Termination criteria
    // const int MAX_INVENTORY_ROUND = 50;
    const int MAX_NUM_QUERIES     = 100000;     // Stop after MAX_NUM_QUERIES have been sent

    // valid values for Q
    const int Q_VALUE [16][4] =
    {
        {0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1},
        {0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1},
        {1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
        {1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1}
    };

    const bool P_DOWN = false;

    // Duration in us
    const int CW_D         = 250;    // Carrier wave
    const int P_DOWN_D     = 2000;    // power down
    const int T1_D         = 40;//200;    // Time from Interrogator transmission to Tag response (250 us)
    const int T2_D         = 480;    // Time from Tag response to Interrogator transmission. Max value = 20.0 * T_tag = 500us
    const int PW_D         = 12;      // Half Tari
    const int DELIM_D       = 12;      // A preamble shall comprise a fixed-length start delimiter 12.5us +/-5%
    const int TRCAL_D     = 200;    // BLF = DR/TRCAL => 40e3 = 8/TRCAL => TRCAL = 200us
    const int RTCAL_D     = 72;      // 6*PW = 72us

    const int NUM_PULSES_COMMAND = 5;       // Number of pulses to detect a reader command
    const int NUMBER_UNIQUE_TAGS = 100;      // Stop after NUMBER_UNIQUE_TAGS have been read


    // Number of bits
    const int PILOT_TONE          = 12;  // Optional
    const int TAG_PREAMBLE_BITS  = 6;   // Number of preamble bits
    const int RN16_BITS          = 17;  // Dummy bit at the end
    const int EPC_BITS            = 129;  // PC + EPC + CRC16 + Dummy = 6 + 16 + 96 + 16 + 1 = 135
    const int QUERY_LENGTH        = 22;  // Query length in bits
    const int EXTRA_BITS          = 10; // extra bits to ungate

    const int T_READER_FREQ = 40000;     // BLF = 40kHz
    const float TAG_BIT_D   = 1.0/T_READER_FREQ * pow(10,6); // Duration in us
    const int RN16_D        = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
    const int EPC_D          = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
    // Query command
    const int QUERY_CODE[4] = {1,0,0,0};
    const int M[2]          = {0,0};
    const int SEL[2]         = {0,0};
    const int SESSION[2]     = {0,0};
    const int TARGET         = 0;
    const int TREXT         = 0;
    const int DR            = 0;


    const int NAK_CODE[8]   = {1,1,0,0,0,0,0,0};

    // ACK command
    const int ACK_CODE[2]   = {0,1};

    // QueryAdjust command
    const int QADJ_CODE[4]   = {1,0,0,1};

    // 110 Increment by 1, 000 unchanged, 010 decrement by 1
    const int Q_UPDN[3][3]  = { {1,1,0}, {0,0,0}, {0,1,0} };

    // FM0 encoding preamble sequences
    // const int TAG_PREAMBLE[] = {1,1,-1,1,-1,-1,1,-1,-1,-1,1,1};
    const int TAG_PREAMBLE[2][2*TAG_PREAMBLE_BITS] =
    {
      {1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1},
      {-1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1}
    };

    // Gate block parameters
    const float THRESH_FRACTION = 0.75;
    const int WIN_SIZE_D         = 250;

    // Duration in which dc offset is estimated (T1_D is 250)
    const int DC_SIZE_D         = 120;

    // Global variable
    extern READER_STATE * reader_state;
    extern void initialize_reader_state();

    // file path
    const std::string result_file_path = "result";
    #ifdef DEBUG_MESSAGE
    const std::string debug_message = "debug_data/";
    #endif

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_GLOBAL_VARS_H */
