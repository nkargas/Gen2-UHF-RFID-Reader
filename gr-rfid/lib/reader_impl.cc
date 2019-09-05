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
#include "reader_impl.h"
#include "rfid/global_vars.h"
#include <sys/time.h>

namespace gr
{
  namespace rfid
  {
    reader::sptr
    reader::make(int sample_rate, int dac_rate)
    {
      return gnuradio::get_initial_sptr
      (new reader_impl(sample_rate,dac_rate));
    }

    /*
    * The private constructor
    */
    reader_impl::reader_impl(int sample_rate, int dac_rate)
    : gr::block("reader",
      gr::io_signature::make( 1, 1, sizeof(float)),
      gr::io_signature::make( 1, 1, sizeof(float)))
    {
      sample_d = 1.0/dac_rate * pow(10,6);

      // Number of samples for transmitting
      n_data0_s = 2 * PW_D / sample_d;
      n_data1_s = 4 * PW_D / sample_d;
      n_pw_s    = PW_D    / sample_d;
      n_cw_s    = CW_D    / sample_d;
      n_delim_s = DELIM_D / sample_d;
      n_trcal_s = TRCAL_D / sample_d;

      // CW waveforms of different sizes
      n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16
      n_cwack_s     = (T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag
      n_p_down_s     = (P_DOWN_D)/sample_d;

      p_down.resize(n_p_down_s);        // Power down samples
      cw_query.resize(n_cwquery_s);      // Sent after query/query rep
      cw_ack.resize(n_cwack_s);          // Sent after ack

      std::fill_n(cw_query.begin(), cw_query.size(), 1);
      std::fill_n(cw_ack.begin(), cw_ack.size(), 1);

      // Construct vectors (resize() default initialization is zero)
      data_0.resize(n_data0_s);
      data_1.resize(n_data1_s);
      cw.resize(n_cw_s);
      delim.resize(n_delim_s);
      rtcal.resize(n_data0_s + n_data1_s);
      trcal.resize(n_trcal_s);

      // Fill vectors with data
      std::fill_n(data_0.begin(), data_0.size()/2, 1);
      std::fill_n(data_1.begin(), 3*data_1.size()/4, 1);
      std::fill_n(cw.begin(), cw.size(), 1);
      std::fill_n(rtcal.begin(), rtcal.size() - n_pw_s, 1); // RTcal
      std::fill_n(trcal.begin(), trcal.size() - n_pw_s, 1); // TRcal

      // create preamble
      preamble.insert( preamble.end(), delim.begin(), delim.end() );
      preamble.insert( preamble.end(), data_0.begin(), data_0.end() );
      preamble.insert( preamble.end(), rtcal.begin(), rtcal.end() );
      preamble.insert( preamble.end(), trcal.begin(), trcal.end() );

      // create framesync
      frame_sync.insert( frame_sync.end(), delim.begin() , delim.end() );
      frame_sync.insert( frame_sync.end(), data_0.begin(), data_0.end() );
      frame_sync.insert( frame_sync.end(), rtcal.begin() , rtcal.end() );

      // create query rep
      query_rep.insert( query_rep.end(), frame_sync.begin(), frame_sync.end());
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );

      // create nak
      nak.insert( nak.end(), frame_sync.begin(), frame_sync.end());
      nak.insert( nak.end(), data_1.begin(), data_1.end() );
      nak.insert( nak.end(), data_1.begin(), data_1.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );

      gen_query_bits();
      gen_query_adjust_bits();
    }

    void reader_impl::gen_query_bits()
    {
      int num_ones = 0, num_zeros = 0;

      query_bits.resize(0);
      query_bits.insert(query_bits.end(), &QUERY_CODE[0], &QUERY_CODE[4]);

      query_bits.push_back(DR);
      query_bits.insert(query_bits.end(), &M[0], &M[2]);
      query_bits.push_back(TREXT);
      query_bits.insert(query_bits.end(), &SEL[0], &SEL[2]);
      query_bits.insert(query_bits.end(), &SESSION[0], &SESSION[2]);
      query_bits.push_back(TARGET);

      query_bits.insert(query_bits.end(), &Q_VALUE[FIXED_Q][0], &Q_VALUE[FIXED_Q][4]);
      crc_append(query_bits);
    }

    void reader_impl::gen_ack_bits(const float * in)
    {
      ack_bits.resize(0);
      ack_bits.insert(ack_bits.end(), &ACK_CODE[0], &ACK_CODE[2]);
      ack_bits.insert(ack_bits.end(), &in[0], &in[16]);
    }

    void reader_impl::gen_query_adjust_bits()
    {
      query_adjust_bits.resize(0);
      query_adjust_bits.insert(query_adjust_bits.end(), &QADJ_CODE[0], &QADJ_CODE[4]);
      query_adjust_bits.insert(query_adjust_bits.end(), &SESSION[0], &SESSION[2]);
      query_adjust_bits.insert(query_adjust_bits.end(), &Q_UPDN[1][0], &Q_UPDN[1][3]);
    }

    reader_impl::~reader_impl(){}

    void reader_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = 0;
    }

    void reader_impl::transmit(float* out, int* written, std::vector<float> bits)
    {
      memcpy(&out[*written], &bits[0], sizeof(float) * bits.size());
      (*written) += bits.size();
    }

    void reader_impl::transmit_bits(float* out, int* written, std::vector<float> bits)
    {
      for(int i=0 ; i<bits.size() ; i++)
      {
        if(bits[i] == 1) transmit(out, written, data_1);
        else transmit(out, written, data_0);
      }
    }

    int reader_impl::general_work(int noutput_items, gr_vector_int &ninput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
    {
      const float* in = (const float*)input_items[0];
      float* out = (float*)output_items[0];
      int consumed = 0;
      int written = 0;

      if(reader_state->gen2_logic_status != IDLE)
      {
        log.open(log_file_path, std::ios::app);

        if(reader_state->gen2_logic_status == START)
        {
          log << "preamble= " << n_delim_s + n_data0_s + n_data0_s + n_data1_s + n_trcal_s << std::endl;
          log << "frame_sync= " << n_delim_s + n_data0_s + n_data0_s + n_data1_s << std::endl;
          log << "delim= " << n_delim_s << std::endl;
          log << "data_0= " << n_data0_s << std::endl;
          log << "rtcal= " << n_data0_s + n_data1_s << std::endl;
          log << "trcal= " << n_trcal_s << std::endl << std::endl;

          log << "cw_query= " << n_cwquery_s << std::endl;
          log << "cw_ack= " << n_cwack_s << std::endl;
          log << "T1= " << T1_D / sample_d << std::endl;
          log << "T2= " << T2_D / sample_d << std::endl;
          log << "RN16= " << RN16_D / sample_d << std::endl;
          log << "EPC= " << EPC_D / sample_d << std::endl << std::endl;

          transmit(out, &written, cw_ack);

          reader_state->gen2_logic_status = IDLE;
        }
        else if(reader_state->gen2_logic_status == SEND_QUERY)
        {
          log << std::endl << "┌──────────────────────────────────────────────────" << std::endl;
          log << "│ Inventory Round: " << reader_state->reader_stats.cur_inventory_round << " | Slot Number: " << reader_state->reader_stats.cur_slot_number << std::endl;
          std::cout << std::endl << "[" << reader_state->reader_stats.cur_inventory_round << "_" << reader_state->reader_stats.cur_slot_number << "] ";
          reader_state->reader_stats.n_queries_sent +=1;

          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;

          transmit(out, &written, cw);
          transmit(out, &written, preamble);
          gen_query_bits();
          transmit_bits(out, &written, query_bits);
          transmit(out, &written, cw_query);

          log << "│ Send Query | Q= " << FIXED_Q << std::endl;
          log << "├──────────────────────────────────────────────────" << std::endl;
          std::cout << "Query(Q=" << FIXED_Q << ") | ";

          reader_state->gen2_logic_status = IDLE;
        }
        else if(reader_state->gen2_logic_status == SEND_QUERY_REP)
        {
          log << "│ Inventory Round: " << reader_state->reader_stats.cur_inventory_round << " | Slot Number: " << reader_state->reader_stats.cur_slot_number << std::endl;
          std::cout << std::endl << "[" << reader_state->reader_stats.cur_inventory_round << "_" << reader_state->reader_stats.cur_slot_number << "] ";
          reader_state->reader_stats.n_queries_sent +=1;

          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;

          transmit(out, &written, cw);
          transmit(out, &written, query_rep);
          transmit(out, &written, cw_query);
          log << "│ Send QueryRep" << std::endl;
          log << "├──────────────────────────────────────────────────" << std::endl;
          std::cout << "QueryRep | ";


          reader_state->gen2_logic_status = IDLE;
        }
        else if(reader_state->gen2_logic_status == SEND_ACK && ninput_items[0] == RN16_BITS - 1)
        {
          reader_state->reader_stats.n_ack_sent +=1;

          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_EPC;
          reader_state->gate_status    = GATE_SEEK_EPC;

          transmit(out, &written, cw);
          transmit(out, &written, frame_sync);
          gen_ack_bits(in);
          transmit_bits(out, &written, ack_bits);
          transmit(out, &written, cw_ack);

          reader_state->reader_stats.ack_sent.push_back((std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str());
          log << "│ Send ACK" << std::endl;
          log << "├──────────────────────────────────────────────────" << std::endl;
          std::cout << "ACK | ";

          consumed = ninput_items[0];
          reader_state->gen2_logic_status = IDLE;
        }
        log.close();
      }

      consume_each (consumed);
      return written;
    }

    int reader_impl::calc_usec(const struct timeval start, const struct timeval end)
    {
      int sec = end.tv_sec - start.tv_sec;
      int usec = sec * 1e6;
      return usec + end.tv_usec - start.tv_usec;
    }

    void reader_impl::print_results()
    {
      std::ofstream result(result_file_path, std::ios::out);

      result << std::endl << "┌──────────────────────────────────────────────────" << std::endl;
      result << "│ Number of QUERY/QUERYREP sent: " << reader_state->reader_stats.n_queries_sent << std::endl;
      result << "│ Number of ACK sent: " << reader_state->reader_stats.n_ack_sent << std::endl;
      result << "│ ";
      for(int i=0 ; i<reader_state->reader_stats.ack_sent.size() ; i++)
        result << reader_state->reader_stats.ack_sent[i] << " ";
      result << std::endl << "│ Current Inventory round: " << reader_state->reader_stats.cur_inventory_round << std::endl;
      result << "├──────────────────────────────────────────────────" << std::endl;
      result << "│ Number of correctly decoded EPC: " << reader_state->reader_stats.n_epc_correct << std::endl;
      result << "│ Number of unique tags: " << reader_state->reader_stats.tag_reads.size() << std::endl;

      if(reader_state->reader_stats.tag_reads.size())
      {
        result << "├───────────────┬──────────────────────────────────" << std::endl;
        result << "│ Tag ID\t│ Num of reads" << std::endl;
        result << "├───────────────┼──────────────────────────────────" << std::endl;
      }

      std::map<int,int>::iterator it;
      for(it = reader_state->reader_stats.tag_reads.begin(); it != reader_state->reader_stats.tag_reads.end(); it++)
        result << "│ " << it->first << "\t\t" << "│ " << it->second << std::endl;

      if(reader_state->reader_stats.tag_reads.size())
        result << "├───────────────┴──────────────────────────────────" << std::endl;
      else
        result << "├──────────────────────────────────────────────────" << std::endl;

      gettimeofday (&reader_state-> reader_stats.end, NULL);
      int execution_time = calc_usec(reader_state->reader_stats.start, reader_state->reader_stats.end);
      result << "│ Execution time: " << execution_time << " (μs)" << std::endl;
      result << "│ Throughput(EPC): " << (double)reader_state->reader_stats.n_epc_correct * (EPC_BITS - 1) / execution_time * 1e6 << " (bits/second)" << std::endl;
      result << "└──────────────────────────────────────────────────" << std::endl;

      result.close();
    }

    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    void reader_impl::crc_append(std::vector<float> & q)
    {
      int crc[] = {1,0,0,1,0};

      for(int i = 0; i < 17; i++)
      {
        int tmp[] = {0,0,0,0,0};
        tmp[4] = crc[3];
        if(crc[4] == 1)
        {
          if (q[i] == 1)
          {
            tmp[0] = 0;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            tmp[3] = crc[2];
          }
          else
          {
            tmp[0] = 1;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            if(crc[2] == 1)
            {
              tmp[3] = 0;
            }
            else
            {
              tmp[3] = 1;
            }
          }
        }
        else
        {
          if (q[i] == 1)
          {
            tmp[0] = 1;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            if(crc[2] == 1)
            {
              tmp[3] = 0;
            }
            else
            {
              tmp[3] = 1;
            }
          }
          else
          {
            tmp[0] = 0;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            tmp[3] = crc[2];
          }
        }
        memcpy(crc, tmp, 5*sizeof(float));
      }
      for (int i = 4; i >= 0; i--)
        q.push_back(crc[i]);
    }
  }
}
