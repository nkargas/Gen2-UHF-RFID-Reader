/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "tag_decoder_impl.h"

namespace gr
{
  namespace rfid
  {
    void tag_decoder_impl::sample_information::makeLog_init(void)
    {
      if(make_detailed_log)
      {
        _detailed_log << "cur_inventory_round= " << reader_state->reader_stats.cur_inventory_round << std::endl;
        _detailed_log << "cur_slot_number= " << reader_state->reader_stats.cur_slot_number << std::endl << std::endl;
        if(_mode == 1) _detailed_log << "##### DECODER_DECODE_RN16 #####" << std::endl;
        else if(_mode == 2) _detailed_log << "##### DECODER_DECODE_EPC #####" << std::endl;
        _detailed_log << "n_samples_to_ungate= " << reader_state->n_samples_to_ungate << std::endl;
        _detailed_log << "ninput_items[0]= " << _total_size << std::endl;
      }
    }

    void tag_decoder_impl::sample_information::makeLog_preamble(void)
    {
      if(make_log) _log << std::endl;
      
      if(_index == -1)
      {
        if(make_log)
          _log << "Preamble detection fail.." << std::endl;
        if(make_detailed_log)
        _detailed_log << "Preamble detection fail" << std::endl << std::endl;
      }
      else
      {
        if(make_log)
          _log << "Preamble detected!" << std::endl;
      }
    }

    void tag_decoder_impl::sample_information::makeLog_RN16(std::vector<float> RN16_bits)
    {
      if(make_log)
      {
        _log << "RN16=";

        for(int i=0 ; i<RN16_BITS ; i++)
        {
          if(i % 4 == 0) _log << " ";
          _log << RN16_bits[i];
        }
      }
      if(make_detailed_log)
      {
        _detailed_log << "RN16= ";

        for(int i=0 ; i<RN16_BITS ; i++)
        {
          if(i % 4 == 0) _detailed_log << " ";
          _detailed_log << RN16_bits[i];
        }

        _detailed_log << std::endl << std::endl;
      }
    }

    void tag_decoder_impl::sample_information::makeLog_EPC(std::vector<float> EPC_bits, int tag_id)
    {
      if(make_log)
      {
        _log << "EPC=";

        for(int i=0 ; i<EPC_BITS ; i++)
        {
          if(i % 4 == 0) _log << " ";
          _log << EPC_bits[i];
          if(i % 16 == 15) _log << std::endl << "    ";
        }

        if(tag_id == -1) _log << "CRC check fail.." << std::endl;
        else _log << "CRC check success! Tag ID= " << tag_id << std::endl;
      }
      if(make_detailed_log)
      {
        _detailed_log << "EPC= ";

        for(int i=0 ; i<RN16_BITS ; i++)
        {
          if(i % 4 == 0) _detailed_log << " ";
          _detailed_log << EPC_bits[i];
          if(i % 16 == 15) _detailed_log << std::endl << "    ";
        }

        if(tag_id == -1) _detailed_log << "CRC check fail" << std::endl << std::endl;
        else _detailed_log << " Tag ID= " << tag_id << std::endl << std::endl;
      }
    }

    void tag_decoder_impl::sample_information::makeLog_nextSlot(void)
    {
      if(make_log)
      {
        _log << "##################################################" << std::endl;
      }
    }

    void tag_decoder_impl::sample_information::makeLog_tagSync(float threshold, float max_corr, int max_index, int win_size)
    {
      if(make_detailed_log)
      {
        _detailed_log << "threshold= " << threshold << std::endl;
        _detailed_log << "corr= " << max_corr << std::endl;
        _detailed_log << "preamble index= " << max_index << std::endl;
        _detailed_log << "sample index= " << max_index + win_size << std::endl;
      }
    }

    void tag_decoder_impl::sample_information::makeLog_tagDetection(int i, float max_corr, float curr_shift, int shift, int max_index, int mask_level)
    {
      if(make_detailed_log)
      {
        _detailed_log << "[" << i+1 << "th bit]\tcorr=";
        _detailed_log << std::left << std::setw(8) << max_corr;
        _detailed_log << "\tcurr_shift=" << curr_shift << "\tshift=";
        _detailed_log << std::left << std::setw(5) << shift;
        _detailed_log << "\tdecoded_bit=" << max_index;
        if(mask_level) _detailed_log << " (high start)" << std::endl;
        else _detailed_log << " (low start)" << std::endl;
      }
    }
  }
}
