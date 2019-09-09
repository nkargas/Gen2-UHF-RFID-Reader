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

#ifndef INCLUDED_RFID_GATE_IMPL_H
#define INCLUDED_RFID_GATE_IMPL_H

#include <rfid/gate.h>
#include <vector>
#include "rfid/global_vars.h"
#include <fstream>

//#define DEBUG_GATE_IMPL_SAMPLE

namespace gr {
  namespace rfid {
    class gate_impl : public gate
    {
      private:
        class gate_log
        {
          private:
            std::ofstream _log;
            std::string _round_slot;
#ifdef DEBUG_GATE_IMPL_SAMPLE
            std::ofstream _detailed_log;
#endif

          public:
            gate_log();
            gate_log(std::string);
            ~gate_log();

            void makeLog_init(int, gr_complex);
            void makeLog(float);
            void makeLog_seek(void);
            void makeLog_readCWFinish(gr_complex, float, float);
            void makeLog_trackFinish(void);
            void makeLog_readyFinish(void);
            void makeLog_nextSlot(int);
        };

        enum SIGNAL_STATE {NEG_EDGE, POS_EDGE};

        int n_samples, n_samples_T1, n_samples_TAG_BIT, n_samples_PW;


        gr_complex avg_cw = gr_complex(0.0,0.0);
        gr_complex avg_dc;

        int max_count;
        int num_pulses;
        bool check;

        float amp_pos_threshold = 0;
        float amp_neg_threshold = 0;


        SIGNAL_STATE signal_state;

       public:
        gate_impl(int sample_rate);
        ~gate_impl();

        void forecast (int noutput_items, gr_vector_int &ninput_items_required);

        int general_work(int noutput_items,
             gr_vector_int &ninput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);

        void gate_fail(int);
    };
  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_GATE_IMPL_H */
