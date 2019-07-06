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
#define MAX_SEARCH_TRACK 5000
#define MAX_SEARCH_READY 3000
#define T1_LEN 200

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
    n_samples(0), win_index(0), dc_index(0), avg_real(0), avg_imag(0), avg_amp(0)
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
          if(std::abs(in[i]) < AMP_LOWBOUND) continue;

          if(++n_samples > 5000)
          {
            log << "n_samples_TAG_BIT= " << n_samples_TAG_BIT << std::endl;
            log << "Average of first 5000 amplitudes= " << avg_real << std::endl;

            reader_state->gen2_logic_status = SEND_QUERY;
            reader_state->gate_status = GATE_CLOSED;
            number_samples_consumed = i-1;
            break;
          }
          else
          {
            avg_real = (avg_real + std::abs(in[i])) / 2;
          }
        }
      }

      // Gate block is controlled by the Gen2 Logic block
      if(reader_state->gate_status == GATE_SEEK_EPC)
      {
        log << "│ Gate seek EPC.." << std::endl;
        reader_state->gate_status = GATE_TRACK;
        n_samples = 0;
        avg_amp = 0;
        max_count = MAX_SEARCH_TRACK;
        trcal = false;
        reader_state->n_samples_to_ungate = (EPC_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
      }
      else if(reader_state->gate_status == GATE_SEEK_RN16)
      {
        log << "│ Gate seek RN16.." << std::endl;
        reader_state->gate_status = GATE_TRACK;
        n_samples = 0;
        avg_amp = 0;
        max_count = MAX_SEARCH_TRACK;
        trcal = true;
        reader_state->n_samples_to_ungate = (RN16_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
      }
      else number_samples_consumed = ninput_items[0]; // GATE_CLOSED

      if(reader_state->gate_status == GATE_TRACK)
      {
        number_samples_consumed = ninput_items[0];
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          if(--max_count <= 0)
          {
            gate_fail();
            number_samples_consumed = i-1;
            break;
          }
          if(std::abs(in[i]) < AMP_LOWBOUND) continue;

          if(n_samples > 50)
          {
            log << "│ Average of first 50 amplitudes= " << avg_amp << std::endl;
            std::cout << "Command found | ";
            reader_state->gate_status = GATE_READY;
            n_samples = 0;
            max_count = MAX_SEARCH_READY;
            number_samples_consumed = i-1;
            break;
          }

          if(std::abs(std::abs(in[i]) - avg_real) > AMP_THRESHOLD)
          {
            avg_amp = (avg_amp + std::abs(std::abs(in[i]) - avg_real)) / 2;
            n_samples++;
          }
        }
      }
      else if(reader_state->gate_status == GATE_READY)
      {
        number_samples_consumed = ninput_items[0];
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          if(--max_count <= 0)
          {
            gate_fail();
            number_samples_consumed = i-1;
            break;
          }
          if(std::abs(in[i]) < AMP_LOWBOUND) continue;


          if(std::abs(std::abs(in[i]) - avg_real) > avg_amp) n_samples = 0;

          if(n_samples++ > T1_LEN)
          {
            if(trcal) {n_samples = 0; trcal = false; continue;}
            log << "│ Gate open!" << std::endl;
            log << "├──────────────────────────────────────────────────" << std::endl;
            reader_state->gate_status = GATE_OPEN;
            n_samples = 0;
            number_samples_consumed = i-1;
            break;
          }
        }
      }
      else if(reader_state->gate_status == GATE_OPEN)
      {
        number_samples_consumed = ninput_items[0];
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          if(n_samples >= reader_state->n_samples_to_ungate)
          {
            reader_state->gate_status = GATE_CLOSED;
            number_samples_consumed = i-1;
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

    void gate_impl::gate_fail(void)
    {
      std::ofstream log;
      log.open(log_file_path, std::ios::app);

      log << std::endl << "│ Gate search FAIL!" << std::endl;
      std::cout << "Gate FAIL!!";
      reader_state->gate_status = GATE_CLOSED;

      reader_state->reader_stats.cur_slot_number++;
      if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
      {
        reader_state->reader_stats.cur_inventory_round ++;
        reader_state->reader_stats.cur_slot_number = 1;

        log << "└──────────────────────────────────────────────────" << std::endl;
        if(reader_state->reader_stats.cur_inventory_round > MAX_NUM_QUERIES)
        {
          reader_state->reader_stats.cur_inventory_round--;
          reader_state->decoder_status = DECODER_TERMINATED;
        }
        else reader_state->gen2_logic_status = SEND_QUERY;
      }
      else
      {
        log << "├──────────────────────────────────────────────────" << std::endl;
        reader_state->gen2_logic_status = SEND_QUERY_REP;
      }

      log.close();
    }
  }
}
