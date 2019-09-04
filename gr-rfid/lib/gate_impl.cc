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

#define AMP_LOWBOUND (0.01) //this will let us find the lowest bound
#define MIN_PULSE (5)
#define T1_LEN (400)
#define FILTER_RATIO (0.9)

#define AMP_POS_THRESHOLD_RATE (0.8)
#define AMP_NEG_THRESHOLD_RATE (0.2)

#define MAX_SEARCH_TRACK (10000)
#define MAX_SEARCH_READY (8000)
#define MAX_SEARCH_SEEK  (4000)

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

      gateOpenTracker.open("gateOpenTracker/"+std::to_string(gateTrackerCount), std::ofstream::app|std::ofstream::binary);


      int number_samples_consumed = ninput_items[0];
      int written = 0;


      log.open(log_file_path, std::ios::app);

      if(reader_state->gate_status != GATE_CLOSED)
      {
        for(int i=0 ; i<ninput_items[0] ; i++)
        {
          gr_complex sample = in[i];
          char data[8];
          memcpy(data, &sample, 8);
          gateOpenTracker.write(data,8);
          

          if(reader_state->gate_status == GATE_START)
          {
            if(++n_samples <= 20000) 
            {
              avg_dc += sample;
            }
            else if(n_samples > 26000)
            {
              avg_dc /= 20000;
              log << "n_samples_TAG_BIT= " << n_samples_TAG_BIT << std::endl;
              log << "Average of first 20000 amplitudes= " << avg_dc << std::endl;

              reader_state->gen2_logic_status = SEND_QUERY;
              reader_state->gate_status = GATE_CLOSED;

              number_samples_consumed = i-1;

              avg_iq = gr_complex(0.0,0.0);
              iq_count = 0;
              n_samples = 0;
              amp_pos_threshold = 0;
              amp_neg_threshold = 0;
              max_count = MAX_SEARCH_SEEK;

              gateTrackerCount++;
              break;
            }
          }
          else if(reader_state->gate_status == GATE_SEEK_RN16)
          {
            log << "│ Gate seek RN16.." << std::endl;
            reader_state->n_samples_to_ungate = (RN16_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
            reader_state->gate_status = GATE_SEEK;
            avg_iq = gr_complex(0.0,0.0);
            iq_count = 0;
            n_samples = 0;
            amp_pos_threshold = 0;
            amp_neg_threshold = 0;
            max_count = MAX_SEARCH_SEEK;
          }
          else if(reader_state->gate_status == GATE_SEEK_EPC)
          {
            log << "│ Gate seek EPC.." << std::endl;
            reader_state->n_samples_to_ungate = (EPC_BITS + TAG_PREAMBLE_BITS + EXTRA_BITS) * n_samples_TAG_BIT;
            reader_state->gate_status = GATE_SEEK;
            avg_iq = gr_complex(0.0,0.0);
            iq_count = 0;
            n_samples = 0;
            amp_pos_threshold = 0;
            amp_neg_threshold = 0;
            max_count = MAX_SEARCH_SEEK;
          }

          sample -= avg_dc;

          if(reader_state->gate_status == GATE_SEEK)
          {
            n_samples++;
            if(--max_count <= 0){
              gate_fail();
              number_samples_consumed = i-1;
              break;
            }else if(abs(sample) < amp_neg_threshold){
              log << "| AVG amp : " <<avg_iq<<std::endl;
              log << "| FIND first neg amp"<<std::endl;
              reader_state->gate_status = GATE_TRACK;
              signal_state = NEG_EDGE;
              num_pulses = 0;
              max_count = MAX_SEARCH_TRACK;

            }else if(n_samples < T1_LEN){
              //add for average iq amplitude
              iq_count++;
              avg_iq += sample;
            }else if(n_samples == T1_LEN){
              //get average iq amplitude in here
              avg_iq /= iq_count;
              log << "| First AVG amp : " <<avg_iq<<std::endl;

              amp_pos_threshold = abs(avg_iq) * AMP_POS_THRESHOLD_RATE;
              amp_neg_threshold = abs(avg_iq) * AMP_NEG_THRESHOLD_RATE;
            }else{// == if(n_samples > T1_LEN)
              //get average iq amplitude in here
              avg_iq = (avg_iq * (float)FILTER_RATIO) + (sample * (float)(1.0 - FILTER_RATIO));
              amp_pos_threshold = abs(avg_iq) * AMP_POS_THRESHOLD_RATE;
              amp_neg_threshold = abs(avg_iq) * AMP_NEG_THRESHOLD_RATE;
            }
          }
          else if(reader_state->gate_status == GATE_TRACK)
          {
            if(--max_count <= 0)
            {//log<<std::endl;
              log<<"GATE TRACK"<<std::endl;
              log<<"abs value : "<<abs(sample)<<std::endl;
              if(signal_state == POS_EDGE) log<<"signal_state : POS_EDGE"<<std::endl;
              else if(signal_state == NEG_EDGE)  log<<"signal_state : NEG_EDGE"<<std::endl;
              log<<"num pulse : "<<num_pulses<<std::endl;
              gate_fail();
              number_samples_consumed = i-1;
              break;
            }//og<<sample<<" ";

            if((signal_state == NEG_EDGE) && (abs(sample) > amp_pos_threshold))
            {
              signal_state = POS_EDGE;
            }
            else if((signal_state == POS_EDGE) && (abs(sample) < amp_neg_threshold))
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

            if(abs(sample) < amp_neg_threshold){ 
              n_samples = 0;
            }else if(n_samples++ > T1_LEN)
            {//log<<std::endl;
              log << "│ Gate open!" << std::endl;
              log << "├──────────────────────────────────────────────────" << std::endl;

              reader_state->gate_status = GATE_OPEN;
              n_samples = 0;
              continue;
            } 
          }
          else if(reader_state->gate_status == GATE_OPEN)
          {
            if(++n_samples > reader_state->n_samples_to_ungate)
            {
              reader_state->gate_status = GATE_CLOSED;
              gateTrackerCount++;
              number_samples_consumed = i-1;

              break;
            }
            out[written++] = sample - avg_iq;
          }
        }
      }

      log.close();
      gateOpenTracker.close();

      consume_each(number_samples_consumed);
      return written;
    }

    void gate_impl::gate_fail(void)
    {
      log.open(log_file_path, std::ios::app);
      gateTrackerCount++;

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

      log.close();
    }
  }
}
