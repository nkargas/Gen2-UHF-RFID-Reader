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

#ifndef INCLUDED_RFID_READER_IMPL_H
#define INCLUDED_RFID_READER_IMPL_H

#include <rfid/reader.h>
#include <vector>
#include <queue>
#include <fstream>

namespace gr
{
  namespace rfid
  {
    class reader_impl : public reader
    {
      private:
        class reader_log
        {
          private:
            std::ofstream _log;

          public:
            reader_log();
            ~reader_log();

            void makeLog_init(int, int, int, int, int, int, int);
            void makeLog_query(bool);
            void makeLog_ack(void);
        };

        int s_rate, d_rate,  n_cwquery_s,  n_cwack_s,n_p_down_s;
        float sample_d, n_data0_s, n_data1_s, n_cw_s, n_pw_s, n_delim_s, n_trcal_s;
        std::vector<float> data_0, data_1, cw, cw_ack, cw_query, delim, frame_sync, preamble, rtcal, trcal, query_bits, ack_bits, query_rep,nak, query_adjust_bits,p_down;
        int q_change; // 0-> increment, 1-> unchanged, 2-> decrement

        void gen_query_bits();
        void gen_ack_bits(const float * in);
        void gen_query_adjust_bits();
        void crc_append(std::vector<float> & q);

        int calc_usec(const struct timeval start, const struct timeval end);
        void print_results();

        void transmit(float*, int*, std::vector<float>);
        void transmit_bits(float*, int*, std::vector<float>);

      public:
        reader_impl(int sample_rate, int dac_rate);
        ~reader_impl();
        void forecast(int, gr_vector_int&);
        int general_work(int, gr_vector_int&, gr_vector_const_void_star&, gr_vector_void_star&);
    };
  }
}

#endif
