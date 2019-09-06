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

    gate_impl::gate_log::~gate_log()
    {
      if(make_log) _log.close();
    }

    void gate_impl::gate_log::makeLog_init(int n_samples_TAG_BIT, gr_complex avg_dc)
    {
      if(make_log)
      {
        _log << "n_samples_TAG_BIT= " << n_samples_TAG_BIT << std::endl;
        _log << "Average of first 20000 amplitudes= " << avg_dc << std::endl;
      }
    }

    void gate_impl::gate_log::makeLog_nextSlot(char ch)
    {
      if(make_log)
      {
        _log << ch << "──────────────────────────────────────────────────" << std::endl;
      }
    }
  }
}
