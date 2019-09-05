/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "tag_decoder_impl.h"

namespace gr
{
  namespace rfid
  {
    tag_decoder_impl::sample_information::sample_information()
    {
      _in = NULL;
      _total_size = 0;
      _mode = -1;
      _index = 0;
      _norm_in.clear();

      _corr = 0;

      log = NULL;
      make_log = false;
    }

    tag_decoder_impl::sample_information::sample_information(gr_complex* __in, int __total_size, int __mode, std::string __round_slot, bool __make_log, bool __make_detailed_log)
    // mode: 0:RN16, 1:EPC
    {
      _in = __in;
      _total_size = __total_size;
      _mode = __mode;
      _index = 0;
      _norm_in.clear();
      for(int i=0 ; i<_total_size ; i++)
        _norm_in.push_back(std::sqrt(std::norm(_in[i])));

      _corr = 0;
      _complex_corr = std::complex<float>(0.0,0.0);

      _round_slot = __round_slot
      if(make_log) _log.open(log_file_path, std::ios::app);
      if(make_detailed_log) _detailed_log.open((debug_folder_path+"log/"+_round_slot).c_str(), std::ios::app);
    }

    tag_decoder_impl::sample_information::~sample_information()
    {
      if(make_log) _log.close();
      if(make_detailed_log) _detailed_log.close();
    }

    void tag_decoder_impl::sample_information::set_index(int __index)
    {
      _index = __index;
    }

    void tag_decoder_impl::sample_information::set_corr(float __corr)
    {
      _corr = __corr;
    }

    void tag_decoder_impl::sample_information::set_complex_corr(gr_complex __complex_corr)
    {
      _complex_corr = __complex_corr;
    }

    gr_complex tag_decoder_impl::sample_information::in(int index)
    {
      return _in[index];
    }

    int tag_decoder_impl::sample_information::total_size(void)
    {
      return _total_size;
    }

    int tag_decoder_impl::sample_information::mode(void)
    {
      return _mode;
    }

    float tag_decoder_impl::sample_information::norm_in(int index)
    {
      return _norm_in[index];
    }

    int tag_decoder_impl::sample_information::index(void)
    {
      return _index;
    }

    float tag_decoder_impl::sample_information::corr(void)
    {
      return _corr;
    }

    gr_complex tag_decoder_impl::sample_information::complex_corr(void)
    {
      return _complex_corr;
    }

    std::string tag_decoder_impl::sample_information::round_slot(void)
    {
      return _round_slot;
    }
  }
}
