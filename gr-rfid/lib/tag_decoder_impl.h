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

#ifndef INCLUDED_RFID_TAG_DECODER_IMPL_H
#define INCLUDED_RFID_TAG_DECODER_IMPL_H

#include <rfid/tag_decoder.h>
#include <vector>
#include "rfid/global_vars.h"
#include <time.h>
#include <numeric>
#include <fstream>

//#define DEBUG_TAG_DECODER_IMPL_INPUT
//#define DEBUG_TAG_DECODER_IMPL_PREAMBLE
//#define DEBUG_TAG_DECODER_IMPL_SAMPLE

namespace gr
{
  namespace rfid
  {
    class tag_decoder_impl : public tag_decoder
    {
      private:
        float n_samples_TAG_BIT;
        int s_rate;
        char * char_bits;

        class sample_information
        {
          private:
            gr_complex* _in;
            int _total_size;
            int _mode;
            std::vector<float> _norm_in;

            int _index;
            float _corr;
            gr_complex _complex_corr;

            std::string _round_slot;
            std::ofstream _log;
            std::ofstream _detailed_log;

          public:
            sample_information();
            sample_information(gr_complex*, int, int, std::string);
            ~sample_information();

            void set_index(int);
            void set_corr(float);
            void set_complex_corr(gr_complex);

            gr_complex in(int);
            int total_size(void);
            int mode(void);
            float norm_in(int);

            int index(void);
            float corr(void);
            gr_complex complex_corr(void);

            std::string round_slot(void);

            // log
            void makeLog_init(void);
            void makeLog_preamble(void);
            void makeLog_RN16(std::vector<float>);
            void makeLog_EPC(std::vector<float>, int);
            void makeLog_nextSlot(void);
            void makeLog_tagSync(float, float, int, int);
            void makeLog_tagDetection(int, float, float, int, int, int);
        };

        // tag_decoder_impl.cc
        bool detect_preamble(sample_information*);
        void decode_RN16(sample_information*, float*);
        void decode_EPC(sample_information*);
        void goto_next_slot(sample_information*);
        int check_crc(char*, int);

        // tag_decoder_decoder.cc
        int tag_sync(sample_information*, int);
        std::vector<float> tag_detection(sample_information*, int, int);
        int determine_first_mask_level(sample_information*, int);
        int decode_single_bit(sample_information* in, int, int, int);

        // debug_message
        void debug_etc(sample_information*);
        void debug_etc_execute(sample_information*, std::string, int, int);

      public:
        tag_decoder_impl(int, std::vector<int>);
        ~tag_decoder_impl();
        void forecast(int, gr_vector_int&);
        int general_work(int, gr_vector_int&, gr_vector_const_void_star&, gr_vector_void_star&);
    };
  }
}

#endif
