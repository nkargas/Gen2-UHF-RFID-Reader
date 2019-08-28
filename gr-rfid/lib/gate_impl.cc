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

#define AMP_LOWBOUND 0.001
#define AMP_POS_THRESHOLD 0.01
#define AMP_NEG_THRESHOLD 0.001
#define MIN_PULSE 5
#define T1_LEN 400

#define MAX_SEARCH_TRACK 10000
#define MAX_SEARCH_READY 8000

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
    n_samples(0), avg_amp(0,0), num_pulses(0)
    {
      n_samples_T1       = T1_D       * (sample_rate / pow(10,6));
      n_samples_TAG_BIT  = TPRI_D  * (sample_rate / pow(10,6));
      n_samples_PW       = PW_D  * (sample_rate / pow(10,6));

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
      time.open("time.csv", std::ios::app);

//      log<<std::endl<<ninput_items[0]<<std::endl;

      number_samples_consumed = ninput_items[0];
      if(reader_state->gate_status != GATE_CLOSED)
      {
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          gr_complex sample = in[i];

          if(reader_state->gate_status == GATE_START)
          {
            if(++n_samples <= 20000) avg_amp += sample;
            else
            {
              avg_amp /= 20000;
              log << "n_samples_TAG_BIT= " << n_samples_TAG_BIT << std::endl;
              log << "Average of first 20000 amplitudes= " << avg_amp << std::endl;

              reader_state->gen2_logic_status = SEND_QUERY;
              reader_state->gate_status = GATE_CLOSED;
              number_samples_consumed = i-1;
              break;
            }
          }
          else if(reader_state->gate_status == GATE_SEEK_RN16)
          {
            log << "│ Gate seek RN16.." << std::endl;
            reader_state->n_samples_to_ungate = (RN16_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
            reader_state->gate_status = GATE_SEEK;
          }
          else if(reader_state->gate_status == GATE_SEEK_EPC)
          {
            log << "│ Gate seek EPC.." << std::endl;
            reader_state->n_samples_to_ungate = (EPC_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
            reader_state->gate_status = GATE_SEEK;
          }


          if(reader_state->gate_status == GATE_SEEK)
          {
            reader_state->gate_status = GATE_TRACK;
            signal_state = NEG_EDGE;
            num_pulses = 0;
            max_count = MAX_SEARCH_TRACK;
          }
          else if(reader_state->gate_status == GATE_TRACK)
          {
            if(--max_count <= 0)
            {//log<<std::endl;
              log<<"GATE TRACK"<<std::endl;
              log<<"abs value : "<<abs(sample - avg_amp)<<std::endl;
              if(signal_state == POS_EDGE) log<<"signal_state : POS_EDGE"<<std::endl;
              else if(signal_state == NEG_EDGE)  log<<"signal_state : NEG_EDGE"<<std::endl;
              log<<"num pulse : "<<num_pulses<<std::endl;
              gate_fail();
              number_samples_consumed = i-1;
              break;
            }//og<<sample<<" ";
            if(abs(sample) < AMP_LOWBOUND) continue;

            if((signal_state == NEG_EDGE) && (abs(sample - avg_amp) > AMP_POS_THRESHOLD))
            {
              signal_state = POS_EDGE;
            }
            else if((signal_state == POS_EDGE) && (abs(sample - avg_amp) < AMP_NEG_THRESHOLD))
            {
              signal_state = NEG_EDGE;
              if(++num_pulses > MIN_PULSE)
              {//log<<std::endl;
                log << "│ Gate ready!" << std::endl;
                std::cout << "Command found | ";
                reader_state->gate_status = GATE_READY;
                n_samples = 0;
                max_count = MAX_SEARCH_READY;
              }
            }
          }
          else if(reader_state->gate_status == GATE_READY)
          {
            if(--max_count <= 0)
            {//log<<std::endl;
              log<<"GATE READY"<<std::endl;
              gate_fail();
              number_samples_consumed = i-1;
              break;
            }//log<<sample<<" ";
//            if(abs(sample) < AMP_LOWBOUND) continue;

            if(abs(sample - avg_amp) < AMP_NEG_THRESHOLD){ 
              n_samples = 0;
              iq_count = 0;
              avg_iq = gr_complex(0.0,0.0);
            }else if(n_samples++ > T1_LEN)
            {//log<<std::endl;
              log << "│ Gate open!" << std::endl;
              log << "├──────────────────────────────────────────────────" << std::endl;
              avg_iq /= (iq_count);

              log<<iq_count<<std::endl;


              reader_state->gate_status = GATE_OPEN;
              n_samples = 0;
              number_samples_consumed = i-1;
              break;
            }else if(n_samples > (n_samples_PW * 2)){
              iq_count++;
              avg_iq += in[i];
            }else{
              log<<n_samples<<", "<<n_samples_PW<<std::endl;
            }
          }
          else if(reader_state->gate_status == GATE_OPEN)
          {
            if(++n_samples > reader_state->n_samples_to_ungate)
            {
              log<<avg_iq<<std::endl;
              reader_state->gate_status = GATE_CLOSED;
              number_samples_consumed = i-1;
              struct timeval tv;
              gettimeofday(&tv,NULL);
              long long time_usec = tv.tv_sec *1000000 + tv.tv_usec - 1566000000000000;
              time<<time_usec<<std::endl;

              break;
            }
            out[written++] = in[i] - avg_iq;
          }
        }
      }

      log.close();
      time.close();
      consume_each(number_samples_consumed);
      return written;
    }

    void gate_impl::gate_fail(void)
    {
      std::ofstream log;
      log.open(log_file_path, std::ios::app);
      ipc.send_failed();

      log << "│ Gate search FAIL!" << std::endl;
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
      time<<0<<std::endl;

      time.close();
      log.close();
    }
  }
}
