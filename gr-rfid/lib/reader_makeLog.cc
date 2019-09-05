/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "reader_impl.h"

namespace gr
{
  namespace rfid
  {
    void reader_impl::reader_log()
    {
      if(make_log) _log.open(log_file_path, std::ios::app);
    }

    void reader_impl::~reader_log()
    {
      if(make_log) _log.close();
    }

    void reader_impl::reader_log::makeLog_init(int n_delim_s, int n_data0_s, int n_data1_s, int n_trcal_s, int n_cwquery_s, int n_cwack_s, int sample_d)
    {
      if(make_log)
      {
        _log << "preamble= " << n_delim_s + n_data0_s + n_data0_s + n_data1_s + n_trcal_s << std::endl;
        _log << "frame_sync= " << n_delim_s + n_data0_s + n_data0_s + n_data1_s << std::endl;
        _log << "delim= " << n_delim_s << std::endl;
        _log << "data_0= " << n_data0_s << std::endl;
        _log << "rtcal= " << n_data0_s + n_data1_s << std::endl;
        _log << "trcal= " << n_trcal_s << std::endl << std::endl;

        _log << "cw_query= " << n_cwquery_s << std::endl;
        _log << "cw_ack= " << n_cwack_s << std::endl;
        _log << "T1= " << T1_D / sample_d << std::endl;
        _log << "T2= " << T2_D / sample_d << std::endl;
        _log << "RN16= " << RN16_D / sample_d << std::endl;
        _log << "EPC= " << EPC_D / sample_d << std::endl << std::endl;
      }
    }

    void reader_impl::reader_log::makeLog_query(bool repeat)
    {
      if(make_log)
      {
        if(!repeat) _log << std::endl << "┌──────────────────────────────────────────────────" << std::endl;

        _log << "│ Inventory Round: " << reader_state->reader_stats.cur_inventory_round << " | Slot Number: " << reader_state->reader_stats.cur_slot_number << std::endl;

        if(!repeat) _log << "│ Send Query | Q= " << FIXED_Q << std::endl;
        else _log << "│ Send QueryRep" << std::endl;

        _log << "├──────────────────────────────────────────────────" << std::endl;
      }
    }

    void reader_impl::reader_log::makeLog_ack(void)
    {
      if(make_log)
      {
        _log << "│ Send ACK" << std::endl;
        _log << "├──────────────────────────────────────────────────" << std::endl;
      }
    }
  }
}
