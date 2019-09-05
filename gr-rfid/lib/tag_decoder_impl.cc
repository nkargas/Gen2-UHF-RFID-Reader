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
      char_bits = new char[EPC_BITS];
      n_samples_TAG_BIT = TPRI_D * s_rate / pow(10,6);
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

        sample_information ys((gr_complex*)input_items[0], ninput_items[0], mode,
          (std::to_string(reader_state->reader_stats.cur_inventory_round)+"_"+std::to_string(reader_state->reader_stats.cur_slot_number)).c_str(),
          make_log, make_detailed_log);
        ys.makeLog_init();

        if(detect_preamble(&ys))
        {
          if(mode == 1) decode_RN16(&ys, out);
          else if(mode == 2) decode_EPC(&ys);
        }
        else
        {
          std::cout << "\t\t\t\t\tPreamble FAIL!!";
          goto_next_slot();
        }
        debug_etc(&ys);

        // process for GNU RADIO
        produce(1, ninput_items[0]);
        consumed = reader_state->n_samples_to_ungate;
      }

      consume_each(consumed);
      return WORK_CALLED_PRODUCE;
    }

    bool tag_decoder_impl::detect_preamble(sample_information* ys)
    {
      if(ys->mode() == 1) tag_sync(ys, RN16_BITS);
      else if(ys->mode() == 2) tag_sync(ys, EPC_BITS);

      ys->makeLog_preamble();
      if(ys->index() == -1) return false;
      else return true;
    }

    void tag_decoder_impl::decode_RN16(sample_information* ys, int index, float* out)
    {
      std::vector<float> RN16_bits = tag_detection(ys, index, RN16_BITS);

      // write RN16_bits to the next block
      int written = 0;
      for(int i=0 ; i<RN16_BITS ; i++)
      {
        out[written++] = RN16_bits[i];
      }
      produce(0, written);
      ys->makeLog_RN16(RN16_bits);

      // go to the next state
      std::cout << "RN16 decoded | ";
      reader_state->gen2_logic_status = SEND_ACK;
    }

    void tag_decoder_impl::decode_EPC(sample_information* ys, int index)
    {
      std::vector<float> EPC_bits = tag_detection(ys, index, EPC_BITS);

      // convert EPC_bits from float to char in order to use Buettner's function
      for(int i=0 ; i<EPC_BITS ; i++)
        char_bits[i] = EPC_bits[i] + '0';

      // check CRC
      if(check_crc(char_bits, EPC_BITS) == 1) // success to decode EPC
      {
        // calculate tag_id
        int tag_id = 0;
        for(int i=0 ; i<8 ; i++)
          tag_id += std::pow(2, 7-i) * EPC_bits[104+i];

        ys->makeLog_EPC(EPC_bits, tag_id);
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
        ys->makeLog_EPC(EPC_bits, -1);
        std::cout << "\t\t\t\t\tCRC FAIL!!";
      }

      goto_next_slot();
    }

    void tag_decoder_impl::goto_next_slot(void)
    {
      reader_state->reader_stats.cur_slot_number++;
      if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
      {
        ys->makeLog_nextslot('└');
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
        ys->makeLog_nextslot('├');
        reader_state->gen2_logic_status = SEND_QUERY_REP;
      }
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

    // below is the debug section
    void tag_decoder_impl::debug_etc(sample_information* ys)
    {
#ifdef DEBUG_TAG_DECODER_IMPL_INPUT
      debug_etc_execute(ys, "input", 0, ys->total_size())
#endif
#ifdef DEBUG_TAG_DECODER_IMPL_PREAMBLE
      debug_etc_execute(ys, "preamble", ys->index()-n_samples_TAG_BIT*TAG_PREAMBLE_BITS, 0)
#endif
#ifdef DEBUG_TAG_DECODER_IMPL_SAMPLE
      if(ys->mode() == 1)
        debug_etc_execute(ys, "sample", ys->index(), ys->index()+n_samples_TAG_BIT*RN16_BITS)
      else if(ys->mode() == 2)
        debug_etc_execute(ys, "sample", ys->index(), ys->index()+n_samples_TAG_BIT*EPC_BITS)
#endif
    }

    void tag_decoder_impl::debug_etc_execute(sample_information* ys, std::string str, int start, int end)
    {
      std::string path = debug_folder_path;
      if(ys->mode() == 1) path.append("RN16_");
      else if(ys->mode() == 2) path.append("EPC_");
      else return;
      path.append(str);
      path.append("/");
      path.append(ys->round_slot());

      std::ofstream debug_i((path+"_I").c_str(), std::ios::app);
      std::ofstream debug_q((path+"_Q").c_str(), std::ios::app);
      std::ofstream debug(path, std::ios::app);

      for(int i=start ; i<end ; i++)
      {
        debug_i << ys->in(i).real() << " ";
        debug_q << ys->in(i).imag() << " ";
        debug << ys->norm_in(i) << " ";
      }

      debug_i.close();
      debug_q.close();
      debug.close();
    }
  }
}
