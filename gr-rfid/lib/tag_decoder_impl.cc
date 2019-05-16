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
#include <gnuradio/prefs.h>
#include <gnuradio/math.h>
#include <cmath>
#include <sys/time.h>
#include "tag_decoder_impl.h"

namespace gr
{
  namespace rfid
  {
    tag_decoder::sptr
    tag_decoder::make(int sample_rate)
    {
      std::vector<int> output_sizes;
      output_sizes.push_back(sizeof(float));
      output_sizes.push_back(sizeof(gr_complex));

      return gnuradio::get_initial_sptr(new tag_decoder_impl(sample_rate,output_sizes));
    }

    tag_decoder_impl::tag_decoder_impl(int sample_rate, std::vector<int> output_sizes)
    : gr::block("tag_decoder", gr::io_signature::make(1, 1, sizeof(gr_complex)), gr::io_signature::makev(2, 2, output_sizes)), s_rate(sample_rate)
    {
      char_bits = new char[128];
      n_samples_TAG_BIT = TPRI_D * s_rate / pow(10,6);
      //GR_LOG_INFO(d_logger, "Number of samples of Tag bit : "<< n_samples_TAG_BIT);
    }

    tag_decoder_impl::~tag_decoder_impl(){}

    void tag_decoder_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int tag_decoder_impl::general_work(int noutput_items, gr_vector_int& ninput_items, gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
    {
      float *out = (float *) output_items[0];
      int consumed = 0;

      sample_information ys ((gr_complex*)input_items[0], ninput_items[0]);

      std::ofstream log;
      current_round_slot = (std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str();

      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if(reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {
        log.open(log_file_path, std::ios::app);
        debug_log.open((debug_folder_path+"log/"+current_round_slot).c_str(), std::ios::app);

        debug_log << "cur_inventory_round= " << reader_state->reader_stats.cur_inventory_round << std::endl;
        debug_log << "cur_slot_number= " << reader_state->reader_stats.cur_slot_number << std::endl << std::endl;

        debug_log << "##### DECODER_DECODE_RN16 #####" << std::endl;
        debug_log << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << std::endl;
        debug_log << "ninput_items[0]= " << ninput_items[0] << std::endl;

        #ifdef DEBUG_DECODER_RN16
        {
          debug_decoder_RN16_i.open((debug_folder_path+"RN16_input/"+current_round_slot+"_I").c_str(), std::ios::app);
          debug_decoder_RN16_q.open((debug_folder_path+"RN16_input/"+current_round_slot+"_Q").c_str(), std::ios::app);
          debug_decoder_RN16.open((debug_folder_path+"RN16_input/"+current_round_slot).c_str(), std::ios::app);
          for(int i=0 ; i<ninput_items[0] ; i++)
          {
            debug_decoder_RN16_i << ys.in(i).real() << " ";
            debug_decoder_RN16_q << ys.in(i).imag() << " ";
            debug_decoder_RN16 << ys.norm_in(i) << " ";
          }
          debug_decoder_RN16_i.close();
          debug_decoder_RN16_q.close();
          debug_decoder_RN16.close();
        }
        #endif

        // detect preamble
        int RN16_index = tag_sync(&ys);  //find where the tag data bits start
        #ifdef DEBUG_DECODER_RN16
        {
          debug_decoder_RN16_i.open((debug_folder_path+"RN16_preamble/"+current_round_slot+"_I").c_str(), std::ios::app);
          debug_decoder_RN16_q.open((debug_folder_path+"RN16_preamble/"+current_round_slot+"_Q").c_str(), std::ios::app);
          debug_decoder_RN16.open((debug_folder_path+"RN16_preamble/"+current_round_slot).c_str(), std::ios::app);
          for(int i=-n_samples_TAG_BIT*TAG_PREAMBLE_BITS ; i<0 ; i++)
          {
            debug_decoder_RN16_i << ys.in(RN16_index+i).real() << " ";
            debug_decoder_RN16_q << ys.in(RN16_index+i).imag() << " ";
            debug_decoder_RN16 << ys.norm_in(RN16_index+i) << " ";
          }
          debug_decoder_RN16_i.close();
          debug_decoder_RN16_q.close();
          debug_decoder_RN16.close();

          debug_decoder_RN16_i.open((debug_folder_path+"RN16_sample/"+current_round_slot+"_I").c_str(), std::ios::app);
          debug_decoder_RN16_q.open((debug_folder_path+"RN16_sample/"+current_round_slot+"_Q").c_str(), std::ios::app);
          debug_decoder_RN16.open((debug_folder_path+"RN16_sample/"+current_round_slot).c_str(), std::ios::app);
          for(int i=0 ; i<n_samples_TAG_BIT*(RN16_BITS-1) ; i++)
          {
            debug_decoder_RN16_i << ys.in(RN16_index+i).real() << " ";
            debug_decoder_RN16_q << ys.in(RN16_index+i).imag() << " ";
            debug_decoder_RN16 << ys.norm_in(RN16_index+i) << " ";
          }
          debug_decoder_RN16_i.close();
          debug_decoder_RN16_q.close();
          debug_decoder_RN16.close();
        }
        #endif

        // process for GNU RADIO
        int written_sync = 0;
        for(int i=0 ; i<ninput_items[0] ; i++)
          written_sync++;
        produce(1, written_sync);

        // decode RN16
        if(RN16_index != -1)
        {
          log << "│ Preamble detected!" << std::endl;
          std::vector<float> RN16_bits = tag_detection(&ys, RN16_index, RN16_BITS-1);  // RN16_BITS includes one dummy bit

          // write RN16_bits to the next block
          log << "│ RN16=";
          debug_log << "RN16= ";
          int written = 0;
          for(int i=0 ; i<RN16_bits.size() ; i++)
          {
            out[written++] = RN16_bits[i];

            if(i % 4 == 0)
            {
              log << " ";
              debug_log << " ";
            }
            log << RN16_bits[i];
            debug_log << RN16_bits[i];
          }
          debug_log << std::endl << std::endl;
          produce(0, written);

          // go to the next state
          log << std::endl << "├──────────────────────────────────────────────────" << std::endl;
          std::cout << "RN16 decoded | ";
          reader_state->gen2_logic_status = SEND_ACK;
        }
        else  // fail to detect preamble
        {
          log << "│ Preamble detection fail.." << std::endl;
          debug_log << "Preamble detection fail" << std::endl << std::endl;
          std::cout << "\t\t\t\t\tPreamble FAIL!!";

          reader_state->reader_stats.cur_slot_number++;
          if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
          {
            reader_state->reader_stats.cur_inventory_round ++;
            reader_state->reader_stats.cur_slot_number = 1;

            log << "└──────────────────────────────────────────────────" << std::endl;
            reader_state->gen2_logic_status = SEND_QUERY;
          }
          else
          {
            log << "├──────────────────────────────────────────────────" << std::endl;
            reader_state->gen2_logic_status = SEND_QUERY_REP;
          }
        }

        log.close();
        debug_log.close();

        // process for GNU RADIO
        consumed = reader_state->n_samples_to_ungate;
      }

      // Processing only after n_samples_to_ungate are available and we need to decode an EPC
      else if (reader_state->decoder_status == DECODER_DECODE_EPC && ninput_items[0] >= reader_state->n_samples_to_ungate )
      {
        log.open(log_file_path, std::ios::app);
        debug_log.open((debug_folder_path+"log/"+current_round_slot).c_str(), std::ios::app);

        debug_log << "##### DECODER_DECODE_EPC #####" << std::endl;
        debug_log << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << std::endl;
        debug_log << "ninput_items[0]= " << ninput_items[0] << std::endl;

        #ifdef DEBUG_DECODER_EPC
        {
          debug_decoder_EPC_i.open((debug_folder_path+"EPC_input/"+current_round_slot+"_I").c_str(), std::ios::app);
          debug_decoder_EPC_q.open((debug_folder_path+"EPC_input/"+current_round_slot+"_Q").c_str(), std::ios::app);
          debug_decoder_EPC.open((debug_folder_path+"EPC_input/"+current_round_slot).c_str(), std::ios::app);
          for(int i=0 ; i<ninput_items[0] ; i++)
          {
            debug_decoder_EPC_i << ys.in(i).real() << " ";
            debug_decoder_EPC_q << ys.in(i).imag() << " ";
            debug_decoder_EPC << ys.norm_in(i) << " ";
          }
          debug_decoder_EPC_i.close();
          debug_decoder_EPC_q.close();
          debug_decoder_EPC.close();
        }
        #endif

        // detect preamble
        int EPC_index = tag_sync(&ys);

        #ifdef DEBUG_DECODER_RN16
        {
          debug_decoder_EPC_i.open((debug_folder_path+"EPC_preamble/"+current_round_slot+"_I").c_str(), std::ios::app);
          debug_decoder_EPC_q.open((debug_folder_path+"EPC_preamble/"+current_round_slot+"_Q").c_str(), std::ios::app);
          debug_decoder_EPC.open((debug_folder_path+"EPC_preamble/"+current_round_slot).c_str(), std::ios::app);
          for(int i=-n_samples_TAG_BIT*TAG_PREAMBLE_BITS ; i<0 ; i++)
          {
            debug_decoder_EPC_i << ys.in(EPC_index+i).real() << " ";
            debug_decoder_EPC_q << ys.in(EPC_index+i).imag() << " ";
            debug_decoder_EPC << ys.norm_in(EPC_index+i) << " ";
          }
          debug_decoder_EPC_i.close();
          debug_decoder_EPC_q.close();
          debug_decoder_EPC.close();

          debug_decoder_EPC_i.open((debug_folder_path+"EPC_sample/"+current_round_slot+"_I").c_str(), std::ios::app);
          debug_decoder_EPC_q.open((debug_folder_path+"EPC_sample/"+current_round_slot+"_Q").c_str(), std::ios::app);
          debug_decoder_EPC.open((debug_folder_path+"EPC_sample/"+current_round_slot).c_str(), std::ios::app);
          for(int i=0 ; i<n_samples_TAG_BIT*(EPC_BITS-1) ; i++)
          {
            debug_decoder_EPC_i << ys.in(EPC_index+i).real() << " ";
            debug_decoder_EPC_q << ys.in(EPC_index+i).imag() << " ";
            debug_decoder_EPC << ys.norm_in(EPC_index+i) << " ";
          }
          debug_decoder_EPC_i.close();
          debug_decoder_EPC_q.close();
          debug_decoder_EPC.close();
        }
        #endif

        // process for GNU RADIO
        int written_sync = 0;
        for(int j=0 ; j<ninput_items[0] ; j++)
          written_sync++;
        produce(1, written_sync);

        // decode EPC
        if(EPC_index != -1)
        {
          log << "│ Preamble detected!" << std::endl;

          std::vector<float> EPC_bits = tag_detection(&ys, EPC_index, EPC_BITS-1);  // EPC_BITS includes one dummy bit

          // convert EPC_bits from float to char in order to use Buettner's function
          log << "│ EPC=";
          debug_log << "EPC=";
          for(int i=0 ; i<EPC_bits.size() ; i++)
          {
            if(i % 4 == 0)
            {
              log << " ";
              debug_log << " ";
            }
            log << EPC_bits[i];
            debug_log << EPC_bits[i];
            char_bits[i] = EPC_bits[i] + '0';
            if(i % 16 == 15)
            {
              log << std::endl << "│     ";
              debug_log << std::endl << "    ";
            }
          }

          // check CRC
          if(check_crc(char_bits, 128) == 1) // success to decode EPC
          {
            // calculate tag_id
            int tag_id = 0;
            for(int i=0 ; i<8 ; i++)
              tag_id += std::pow(2, 7-i) * EPC_bits[104+i];

            //GR_LOG_INFO(d_debug_logger, "EPC CORRECTLY DECODED, TAG ID : " << tag_id);
            log << "CRC check success! Tag ID= " << tag_id << std::endl;
            debug_log << " Tag ID= " << tag_id << std::endl << std::endl;
            std::cout << "\t\t\t\t\t\t\t\t\t\tTag ID= " << tag_id;
            reader_state->reader_stats.n_epc_correct+=1;

            // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
            std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(tag_id);
            if ( it != reader_state->reader_stats.tag_reads.end())
              it->second ++;
            else
              reader_state->reader_stats.tag_reads[tag_id]=1;
          }
          else
          {
            log << "│ CRC check fail.." << std::endl;
            debug_log << "CRC check fail" << std::endl << std::endl;
            std::cout << "\t\t\t\t\tCRC FAIL!!";
          }
        }
        else
        {
          log << "│ Preamble detection fail.." << std::endl;
          debug_log << "Preamble detection fail" << std::endl << std::endl;
          std::cout << "\t\t\t\t\tPreamble FAIL!!";
        }

        // After EPC message send a query rep or query
        reader_state->reader_stats.cur_slot_number++;
        if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
        {
          reader_state->reader_stats.cur_inventory_round ++;
          reader_state->reader_stats.cur_slot_number = 1;

          log << "└──────────────────────────────────────────────────" << std::endl;
          reader_state->gen2_logic_status = SEND_QUERY;
        }
        else
        {
          log << "├──────────────────────────────────────────────────" << std::endl;
          reader_state->gen2_logic_status = SEND_QUERY_REP;
        }
        log.close();
        debug_log.close();

        // process for GNU RADIO
        consumed = reader_state->n_samples_to_ungate;
      }

      consume_each(consumed);
      return WORK_CALLED_PRODUCE;
    }

    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    int tag_decoder_impl::check_crc(char * bits, int num_bits)
    {
      register unsigned short i, j;
      register unsigned short crc_16, rcvd_crc;
      unsigned char * data;
      int num_bytes = num_bits / 8;
      data = (unsigned char* )malloc(num_bytes );
      int mask;

      for(i = 0; i < num_bytes; i++)
      {
        mask = 0x80;
        data[i] = 0;
        for(j = 0; j < 8; j++)
        {
          if (bits[(i * 8) + j] == '1'){
            data[i] = data[i] | mask;
          }
          mask = mask >> 1;
        }
      }
      rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes -1];

      crc_16 = 0xFFFF;
      for (i=0; i < num_bytes - 2; i++)
      {
        crc_16^=data[i] << 8;
        for (j=0;j<8;j++)
        {
          if (crc_16&0x8000)
          {
            crc_16 <<= 1;
            crc_16 ^= 0x1021;
          }
          else
          crc_16 <<= 1;
        }
      }
      crc_16 = ~crc_16;

      if(rcvd_crc != crc_16)
      return -1;
      else
      return 1;
    }
  } /* namespace rfid */
} /* namespace gr */
