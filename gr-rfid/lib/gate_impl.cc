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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "gate_impl.h"
#include <sys/time.h>
#include <stdio.h>

#define FIRST_DC_SAMPLES (20000)
#define AMP_LOWBOUND (0.001)
#define MIN_PULSE (5)
#define CW_LEN (200)
#define T1_LEN (400)
#define FILTER_RATIO (0.9)

#define AMP_POS_THRESHOLD_RATE (0.8)
#define AMP_NEG_THRESHOLD_RATE (0.2)

#define MAX_SEARCH_READ_CW (20000)  // (from) tag command end (to) reader pulse start
#define MAX_SEARCH_TRACK (5000)    // (from) reader pusle start (to) MIN_PULSE'th reader pulse
#define MAX_SEARCH_READY (5000)     // (from) MIN_PULSE'th reader pulse (to) tag command start

namespace gr
{
  namespace rfid
  {
    gate::sptr

    gate::make(int sample_rate)
    {
      return gnuradio::get_initial_sptr(new gate_impl(sample_rate));
    }

    /*
    * The private constructor
    */
    gate_impl::gate_impl(int sample_rate)
    : gr::block("gate",
    gr::io_signature::make(1, 1, sizeof(gr_complex)),
    gr::io_signature::make(1, 1, sizeof(gr_complex))),

    n_samples(0), avg_dc(0,0), num_pulses(0)
    {
      n_samples_T1       = T1_D       * (sample_rate / pow(10,6));
      n_samples_TAG_BIT  = TPRI_D  * (sample_rate / pow(10,6));
      n_samples_PW       = PW_D  * (sample_rate / pow(10,6));

      // First block to be scheduled
      initialize_reader_state();
    }

    gate_impl::~gate_impl(){}

    void gate_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int gate_impl::general_work (int noutput_items, gr_vector_int &ninput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      int number_samples_consumed = ninput_items[0];
      int written = 0;

      if(reader_state->gate_status == GATE_START)
      {
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          if(++n_samples <= FIRST_DC_SAMPLES)
            avg_dc += in[i];
          else
          {
            avg_dc /= FIRST_DC_SAMPLES;

            gate_log ys;
            ys.makeLog_init(n_samples_TAG_BIT, avg_dc);

            reader_state->gen2_logic_status = SEND_QUERY;
            reader_state->gate_status = GATE_CLOSED;

            number_samples_consumed = i-1;
            break;
          }
        }
      }
      else if(reader_state->gate_status != GATE_CLOSED)
      {
        gate_log ys((std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str());

        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          gr_complex sample = in[i] - avg_dc;

          if(reader_state->gate_status == GATE_SEEK_RN16)
          {
            reader_state->n_samples_to_ungate = (RN16_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
            reader_state->gate_status = GATE_SEEK;
          }
          else if(reader_state->gate_status == GATE_SEEK_EPC)
          {
            reader_state->n_samples_to_ungate = (EPC_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
            reader_state->gate_status = GATE_SEEK;
          }

          if(reader_state->gate_status == GATE_SEEK)
          {
            ys.makeLog_seek();
            avg_cw = gr_complex(0.0, 0.0);
            n_samples = 0;
            check = false;
            amp_pos_threshold = 0;
            amp_neg_threshold = 0;
            max_count = MAX_SEARCH_READ_CW;
            reader_state->gate_status = GATE_READ_CW;
          }

          if(reader_state->gate_status == GATE_READ_CW)
          {
            ys.makeLog(std::abs(sample));
            if(std::abs(sample) < AMP_LOWBOUND) continue;
            
            if(--max_count <= 0)
            {
              gate_fail(1);
              number_samples_consumed = i-1;
              break;
            }
            else if(std::abs(sample) < amp_neg_threshold)
              // reader command starts
            {
              ys.makeLog_readCWFinish(avg_cw, amp_pos_threshold, amp_neg_threshold);
              signal_state = NEG_EDGE;
              num_pulses = 0;
              max_count = MAX_SEARCH_TRACK;
              reader_state->gate_status = GATE_TRACK;
            }
            else if(++n_samples < CW_LEN)
              // add amplitude for first avg_cw
              avg_cw += sample;
            else if(n_samples == CW_LEN)
              // calculate first avg_cw
            {
              avg_cw /= CW_LEN;
              amp_pos_threshold = std::abs(avg_cw) * AMP_POS_THRESHOLD_RATE;
              amp_neg_threshold = std::abs(avg_cw) * AMP_NEG_THRESHOLD_RATE;
            }
            else
              // update avg_cw with FILTER_RATIO for each sample
              // until reader command starts
            {
              avg_cw = (avg_cw * (float)FILTER_RATIO) + (sample * (float)(1.0 - FILTER_RATIO));
              amp_pos_threshold = std::abs(avg_cw) * AMP_POS_THRESHOLD_RATE;
              amp_neg_threshold = std::abs(avg_cw) * AMP_NEG_THRESHOLD_RATE;
            }
          }
          else if(reader_state->gate_status == GATE_TRACK)
          {
            ys.makeLog(std::abs(sample));
            if(--max_count <= 0)
            {
              gate_fail(2);
              number_samples_consumed = i-1;
              break;
            }

            if((signal_state == NEG_EDGE) && (std::abs(sample) > amp_pos_threshold))
              // neg_edge --> pos_edge
            {
              signal_state = POS_EDGE;
            }
            else if((signal_state == POS_EDGE) && (std::abs(sample) < amp_neg_threshold))
              // pos_edge --> neg_edge (count++)
            {
              signal_state = NEG_EDGE;
              if(++num_pulses > MIN_PULSE)
              {
                ys.makeLog_trackFinish();
                n_samples = 0;
                max_count = MAX_SEARCH_READY;
                reader_state->gate_status = GATE_READY;
              }
            }
          }
          else if(reader_state->gate_status == GATE_READY)
          {
            ys.makeLog(std::abs(sample));
            if(--max_count <= 0)
            {
              gate_fail(3);
              number_samples_consumed = i-1;
              break;
            }

            if(std::abs(sample) < amp_neg_threshold)
              // still exist reader command
            {
              n_samples = 0;
            }
            else if(++n_samples > T1_LEN)
              // reader command finished & wait T1_time
            {
              ys.makeLog_readyFinish();
              n_samples = 0;
              reader_state->gate_status = GATE_OPEN;
              continue;
            }
          }
          else if(reader_state->gate_status == GATE_OPEN)
          {
            ys.makeLog(std::abs(sample));
            if(++n_samples > reader_state->n_samples_to_ungate)
            {
              reader_state->gate_status = GATE_CLOSED;
              number_samples_consumed = i-1;

              break;
            }

            out[written++] = sample - avg_cw;
          }
        }
      }

      consume_each(number_samples_consumed);
      return written;
    }

    void gate_impl::gate_fail(int type)
    {
      gate_log ys;
      ys.makeLog_nextSlot(type);

      if(type == 1) std::cout << "\tPulse detection FAIL!!";
      else if(type == 2) std::cout << "\t\tCommand detection FAIL!!";
      else if(type == 3) std::cout << "\t\t\tToo long command!!";
      reader_state->reader_stats.n_gate_fail++;
      reader_state->gate_status = GATE_CLOSED;

      reader_state->reader_stats.cur_slot_number++;
      if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
      {
        reader_state->reader_stats.cur_inventory_round ++;
        reader_state->reader_stats.cur_slot_number = 1;

        if(reader_state->reader_stats.cur_inventory_round > MAX_NUM_QUERIES)
        {
          reader_state->reader_stats.cur_inventory_round--;
          reader_state->decoder_status = DECODER_TERMINATED;
        }
        else reader_state->gen2_logic_status = SEND_QUERY;
      }
      else
      {
        reader_state->gen2_logic_status = SEND_QUERY_REP;
      }
    }
  }
}
