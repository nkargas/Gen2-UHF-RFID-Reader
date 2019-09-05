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

#define __DEBUG_LOG__


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
            std::vector<float> _norm_in;

            float _corr;
            gr_complex _complex_corr;

          public:
            sample_information();
            sample_information(gr_complex*, int);
            ~sample_information();

            void set_corr(float);
            void set_complex_corr(gr_complex);

            gr_complex in(int);
            int total_size(void);
            float norm_in(int);

            float corr(void);
            gr_complex complex_corr(void);
        };

        // tag_decoder_impl.cc
        void decode_RN16(sample_information*, int, float*);
        void decode_EPC(sample_information*, int);
        void goto_next_slot(void);
        int check_crc(char*, int);

        // tag_decoder_decoder.cc
        int tag_sync(sample_information*, int);
        std::vector<float> tag_detection(sample_information*, int, int);
        int determine_first_mask_level(sample_information*, int);
        int decode_single_bit(sample_information* in, int, int, int);

        // debug_message
        std::string current_round_slot;
#ifdef __DEBUG_LOG__
        std::ofstream log;
        std::ofstream debug_log;
#endif
#ifdef DEBUG_TAG_DECODER_IMPL_INPUT
        void debug_input(sample_information*, int, std::string);
#endif
#ifdef DEBUG_TAG_DECODER_IMPL_PREAMBLE
        void debug_preamble(sample_information*, int, std::string, int);
#endif
#ifdef DEBUG_TAG_DECODER_IMPL_SAMPLE
        void debug_sample(sample_information*, int, std::string, int);
#endif

      public:
        tag_decoder_impl(int, std::vector<int>);
        ~tag_decoder_impl();
        void forecast(int, gr_vector_int&);
        int general_work(int, gr_vector_int&, gr_vector_const_void_star&, gr_vector_void_star&);
    };
  }
}

#endif
