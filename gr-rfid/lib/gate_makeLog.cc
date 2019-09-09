/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gate_impl.h"

namespace gr
{
  namespace rfid
  {
    gate_impl::gate_log::gate_log()
    {
      if(make_log) _log.open(log_file_path, std::ios::app);
    }

    gate_impl::gate_log::gate_log(std::string __round_slot)
    {
      if(make_log) _log.open(log_file_path, std::ios::app);
      _round_slot = __round_slot;
#ifdef DEBUG_GATE_IMPL_SAMPLE
      _detailed_log.open((debug_folder_path+"gate/"+_round_slot).c_str(), std::ios::app);
#endif
    }

    gate_impl::gate_log::~gate_log()
    {
      if(make_log) _log.close();
#ifdef DEBUG_GATE_IMPL_SAMPLE
      _detailed_log.close();
#endif
    }

    void gate_impl::gate_log::makeLog_init(int n_samples_TAG_BIT, gr_complex avg_dc)
    {
      if(make_log)
      {
        _log << "n_samples_TAG_BIT= " << n_samples_TAG_BIT << std::endl;
        _log << "Average of DC amplitudes= " << avg_dc << std::endl;
      }
    }

    void gate_impl::gate_log::makeLog(float sample)
    {
#ifdef DEBUG_GATE_IMPL_SAMPLE
      _detailed_log << sample << " ";
#endif
    }

    void gate_impl::gate_log::makeLog_seek(void)
    {
#ifdef DEBUG_GATE_IMPL_SAMPLE
      _detailed_log << std::endl << std::endl;
#endif
    }

    void gate_impl::gate_log::makeLog_readCWFinish(gr_complex avg_cw, float amp_pos_threshold, float amp_neg_threshold)
    {
      if(make_log)
      {
        _log << std::endl << "avg_cw= " << avg_cw << std::endl;
        _log << "pos_threshold= " << amp_pos_threshold << " / neg_threshold= " << amp_neg_threshold << std::endl;
      }
#ifdef DEBUG_GATE_IMPL_SAMPLE
      _detailed_log << std::endl;
#endif
    }

    void gate_impl::gate_log::makeLog_trackFinish(void)
    {
#ifdef DEBUG_GATE_IMPL_SAMPLE
      _detailed_log << std::endl;
#endif
    }

    void gate_impl::gate_log::makeLog_readyFinish(void)
    {
#ifdef DEBUG_GATE_IMPL_SAMPLE
      _detailed_log << std::endl;
#endif
    }

    void gate_impl::gate_log::makeLog_nextSlot(int type)
    {
      if(make_log)
      {
        if(type == 1) _log << "Pulse detection fail.." << std::endl;
        else if(type == 2) _log << "Reader command detection fail.." << std::endl;
        else if(type == 3) _log << "Reader command is too long.." << std::endl;
        _log << "##################################################" << std::endl;
      }
    }
  }
}
