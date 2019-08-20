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
      ipc.send_sync();
    }




    tag_decoder_impl::~tag_decoder_impl(){}




    void tag_decoder_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }





    int tag_decoder_impl::general_work(int noutput_items, gr_vector_int& ninput_items, gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
    {
      float* out = (float *)output_items[0];
      int consumed = 0;

      // Processing only after n_samples_to_ungate are available and we need to decode
      if(ninput_items[0] >= reader_state->n_samples_to_ungate)
      {
        int mode = -1;
        if(reader_state->decoder_status == DECODER_DECODE_RN16) mode = 1;
        else if(reader_state->decoder_status == DECODER_DECODE_EPC) mode = 2;

        current_round_slot = (std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str();
        sample_information ys ((gr_complex*)input_items[0], ninput_items[0]);

#ifdef __DEBUG_LOG__

        log.open(log_file_path, std::ios::app);
        debug_log.open((debug_folder_path+"log/"+current_round_slot).c_str(), std::ios::app);

        debug_log << "cur_inventory_round= " << reader_state->reader_stats.cur_inventory_round << std::endl;
        debug_log << "cur_slot_number= " << reader_state->reader_stats.cur_slot_number << std::endl << std::endl;
        if(mode == 1) debug_log << "##### DECODER_DECODE_RN16 #####" << std::endl;
        else if(mode == 2) debug_log << "##### DECODER_DECODE_EPC #####" << std::endl;
        debug_log << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << std::endl;
        debug_log << "ninput_items[0]= " << ninput_items[0] << std::endl;
#endif

#ifdef DEBUG_TAG_DECODER_IMPL_INPUT
        debug_input(&ys, mode, current_round_slot);
#endif

        // detect preamble
        int index;
        if(mode == 1) index = tag_sync(&ys, RN16_BITS-1);
        else if(mode == 2) index = tag_sync(&ys, EPC_BITS-1);

        if(index == -1)
        {
#ifdef __DEBUG_LOG__
          log << "│ Preamble detection fail.." << std::endl;
          debug_log << "Preamble detection fail" << std::endl << std::endl;
#endif
          std::cout << "\t\t\t\t\tPreamble FAIL!!";
          ipc.send_failed();
          goto_next_slot();
        }
        else
        {
#ifdef __DEBUG_LOG__
          log << "│ Preamble detected!" << std::endl;
#endif


#ifdef DEBUG_TAG_DECODER_IMPL_PREAMBLE
          debug_preamble(&ys, mode, current_round_slot, index);
#endif

#ifdef DEBUG_TAG_DECODER_IMPL_SAMPLE
          debug_sample(&ys, mode, current_round_slot, index);
#endif

          if(mode == 1) decode_RN16(&ys, index, out);
          else if(mode == 2) decode_EPC(&ys, index);
        }

#ifdef __DEBUG_LOG__
        log.close();
        debug_log.close();
#endif

        // process for GNU RADIO
        produce(1, ninput_items[0]);
        consumed = reader_state->n_samples_to_ungate;
      }

      consume_each(consumed);
      return WORK_CALLED_PRODUCE;
    }



    void tag_decoder_impl::decode_RN16(sample_information* ys, int index, float* out)
    {
      std::vector<float> RN16_bits = tag_detection(ys, index, RN16_BITS-1);  // RN16_BITS includes one dummy bit

      ipc.send_avg_corr(RN16_bits,(double)ys->corr(), ys->complex_corr());

#ifdef __DEBUG_LOG__
      // write RN16_bits to the next block
      log << "│ RN16=";
      debug_log << "RN16= ";
#endif
      int written = 0;
      for(int i=0 ; i<RN16_bits.size() ; i++)
      {
        out[written++] = RN16_bits[i];

#ifdef __DEBUG_LOG__
        if(i % 4 == 0)
        {
          log << " ";
          debug_log << " ";
        }
        log << RN16_bits[i];
        debug_log << RN16_bits[i];

#endif

      }
#ifdef __DEBUG_LOG__
      debug_log << std::endl << std::endl;
#endif

      produce(0, written);

      // go to the next state
#ifdef __DEBUG_LOG__
      log << std::endl << "├──────────────────────────────────────────────────" << std::endl;
#endif

      std::cout << "RN16 decoded | ";
      //reader_state->gen2_logic_status = SEND_ACK;
      
      goto_next_slot();      
    }



    void tag_decoder_impl::decode_EPC(sample_information* ys, int index)
    {
      std::vector<float> EPC_bits = tag_detection(ys, index, EPC_BITS-1);  // EPC_BITS includes one dummy bit

      // convert EPC_bits from float to char in order to use Buettner's function
     
#ifdef __DEBUG_LOG__
     
      log << "│ EPC=";
      debug_log << "EPC=";

#endif

      for(int i=0 ; i<EPC_bits.size() ; i++)
      {
        char_bits[i] = EPC_bits[i] + '0';
#ifdef __DEBUG_LOG__
        if(i % 4 == 0)
        {
          log << " ";
          debug_log << " ";
        }
        log << EPC_bits[i];
        debug_log << EPC_bits[i];
        if(i % 16 == 15)
        {
          log << std::endl << "│     ";
          debug_log << std::endl << "    ";
        }
#endif
      }

      // check CRC
      if(check_crc(char_bits, 128) == 1) // success to decode EPC
      {
        // calculate tag_id
        int tag_id = 0;
        for(int i=0 ; i<8 ; i++)
          tag_id += std::pow(2, 7-i) * EPC_bits[104+i];

#ifdef __DEBUG_LOG__
        log << " CRC check success! Tag ID= " << tag_id << std::endl;
        debug_log << " Tag ID= " << tag_id << std::endl << std::endl;
#endif
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
#ifdef __DEBUG_LOG__
        log << " CRC check fail.." << std::endl;
        debug_log << "CRC check fail" << std::endl << std::endl;
#endif
        std::cout << "\t\t\t\t\tCRC FAIL!!";
      }

      goto_next_slot();
    }

    void tag_decoder_impl::goto_next_slot(void)
    {
      reader_state->reader_stats.cur_slot_number++;
      if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
      {
        reader_state->reader_stats.cur_inventory_round ++;
        reader_state->reader_stats.cur_slot_number = 1;

#ifdef  __DEBUG_LOG__
        log << "└──────────────────────────────────────────────────" << std::endl;
#endif
        if(reader_state->reader_stats.cur_inventory_round > MAX_NUM_QUERIES)
        {
          reader_state->reader_stats.cur_inventory_round--;
          reader_state->decoder_status = DECODER_TERMINATED;
        }
        else reader_state->gen2_logic_status = SEND_QUERY;
      }
      else
      {
#ifdef __DEBUG_LOG__
        log << "├──────────────────────────────────────────────────" << std::endl;
#endif
        reader_state->gen2_logic_status = SEND_QUERY_REP;
      }
    }

#ifdef DEBUG_TAG_DECODER_IMPL_INPUT
    void tag_decoder_impl::debug_input(sample_information* ys, int mode, std::string current_round_slot)
    {
      std::string path;
      if(mode == 1) path = (debug_folder_path+"RN16_input/"+current_round_slot).c_str();
      else if(mode == 2) path = (debug_folder_path+"EPC_input/"+current_round_slot).c_str();
      else return;

      std::ofstream debug_i((path+"_I").c_str(), std::ios::app);
      std::ofstream debug_q((path+"_Q").c_str(), std::ios::app);
      std::ofstream debug(path, std::ios::app);

      for(int i=0 ; i<ys->total_size() ; i++)
      {
        debug_i << ys->in(i).real() << " ";
        debug_q << ys->in(i).imag() << " ";
        debug << ys->norm_in(i) << " ";
      }

      debug_i.close();
      debug_q.close();
      debug.close();
    }
#endif

#ifdef DEBUG_TAG_DECODER_IMPL_PREAMBLE
    void tag_decoder_impl::debug_preamble(sample_information* ys, int mode, std::string current_round_slot, int index)
    {
      std::string path;
      if(mode == 1) path = (debug_folder_path+"RN16_preamble/"+current_round_slot).c_str();
      else if(mode == 2) path = (debug_folder_path+"EPC_preamble/"+current_round_slot).c_str();
      else return;

      std::ofstream debug_i((path+"_I").c_str(), std::ios::app);
      std::ofstream debug_q((path+"_Q").c_str(), std::ios::app);
      std::ofstream debug(path, std::ios::app);

      for(int i=-n_samples_TAG_BIT*TAG_PREAMBLE_BITS ; i<0 ; i++)
      {
        debug_i << ys->in(index+i).real() << " ";
        debug_q << ys->in(index+i).imag() << " ";
        debug << ys->norm_in(index+i) << " ";
      }

      debug_i.close();
      debug_q.close();
      debug.close();
    }
#endif

#ifdef DEBUG_TAG_DECODER_IMPL_SAMPLE
    void tag_decoder_impl::debug_sample(sample_information* ys, int mode, std::string current_round_slot, int index)
    {
      std::string path;
      if(mode == 1) path = (debug_folder_path+"RN16_sample/"+current_round_slot).c_str();
      else if(mode == 2) path = (debug_folder_path+"EPC_sample/"+current_round_slot).c_str();
      else return;

      std::ofstream debug_i((path+"_I").c_str(), std::ios::app);
      std::ofstream debug_q((path+"_Q").c_str(), std::ios::app);
      std::ofstream debug(path, std::ios::app);

      for(int i=0 ; i<n_samples_TAG_BIT*(EPC_BITS-1) ; i++)
      {
        debug_i << ys->in(index+i).real() << " ";
        debug_q << ys->in(index+i).imag() << " ";
        debug << ys->norm_in(index+i) << " ";
      }

      debug_i.close();
      debug_q.close();
      debug.close();
    }
#endif

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
  }
}
