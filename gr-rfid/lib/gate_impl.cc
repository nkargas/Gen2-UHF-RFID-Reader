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

#define AMP_LOWBOUND 0.01
#define AMP_THRESHOLD 0.005

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
    n_samples(0), win_index(0), dc_index(0), num_pulses(0), avg_amp(0)
    {
      n_samples_T1       = T1_D       * (sample_rate / pow(10,6));
      n_samples_PW       = PW_D       * (sample_rate / pow(10,6));
      n_samples_TAG_BIT  = TPRI_D  * (sample_rate / pow(10,6));

      win_length = WIN_SIZE_D * (sample_rate/ pow(10,6));
      dc_length  = DC_SIZE_D  * (sample_rate / pow(10,6));

      win_samples.resize(win_length);
      dc_samples.resize(dc_length);

      // First block to be scheduled
      initialize_reader_state();
    }

    /*
    * Our virtual destructor.
    */
    gate_impl::~gate_impl()
    {
    }

    void
    gate_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    gate_impl::general_work (int noutput_items,
      gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      int number_samples_consumed = 0;
      int written = 0;

      std::ofstream log;
      log.open(log_file_path, std::ios::app);

      if(reader_state->gate_status == GATE_START)
      {
        number_samples_consumed = ninput_items[0];
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          if(std::norm(in[i]) < AMP_LOWBOUND) continue;
          n_samples++;
          if(n_samples > 5000)
          {
            log << "n_samples_TAG_BIT= " << n_samples_TAG_BIT << std::endl;
            log << "Average of first 5000 amplitudes= " << avg_amp << std::endl;
            reader_state->gen2_logic_status = SEND_QUERY;
            reader_state->gate_status = GATE_CLOSED;
            number_samples_consumed = i-1;
            break;
          }
          else avg_amp = (avg_amp + std::norm(in[i])) / 2;
        }
      }

      // Gate block is controlled by the Gen2 Logic block
      if(reader_state->gate_status == GATE_SEEK_EPC)
      {
        reader_state->gate_status = GATE_TRACK;
        n_samples = 0;
        num_pulses = 0;
        reader_state->n_samples_to_ungate = (EPC_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
      }
      else if(reader_state->gate_status == GATE_SEEK_RN16)
      {
        reader_state->gate_status = GATE_TRACK;
        n_samples = 0;
        num_pulses = 0;
        reader_state->n_samples_to_ungate = (RN16_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
      }
      else number_samples_consumed = ninput_items[0];

      if(reader_state->gate_status == GATE_TRACK)
      {
        number_samples_consumed = ninput_items[0];
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          if(std::norm(in[i]) < AMP_LOWBOUND) continue;

          if(std::norm(in[i]) - avg_amp > AMP_THRESHOLD)
          {
            log << "│ Found reader command!" << std::endl;
            reader_state->gate_status = GATE_READY;
            n_samples = 0;
            num_pulses++;
            number_samples_consumed = i-1;
            break;
          }
        }
      }
      else if(reader_state->gate_status == GATE_READY)
      {
        number_samples_consumed = ninput_items[0];
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          if(std::norm(in[i]) < AMP_LOWBOUND) continue;

          if(std::norm(in[i]) - avg_amp > AMP_THRESHOLD)
          {  n_samples = 0; num_pulses++;}
          else
          {
            if(++n_samples > n_samples_T1)
            {
              log << "│ Gate open! | num_pulses= " << num_pulses << std::endl;
              log << "├──────────────────────────────────────────────────" << std::endl;
              reader_state->gate_status = GATE_OPEN;
              n_samples = 0;
              number_samples_consumed = i-1;
              break;
            }
          }
        }
      }
      else if(reader_state->gate_status == GATE_OPEN)
      {
        number_samples_consumed = ninput_items[0];
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          if (n_samples >= reader_state->n_samples_to_ungate)
          {
            reader_state->gate_status = GATE_CLOSED;
            number_samples_consumed = i+1;
            break;
          }
          out[written++] = in[i];
          n_samples++;
        }
      }
      log.close();
      consume_each (number_samples_consumed);
      return written;
    }
  }
}
