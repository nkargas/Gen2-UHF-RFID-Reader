/* -*- c++ -*- */
#ifndef INCLUDED_RFID_TAG_DECODER_IMPL_H
#define INCLUDED_RFID_TAG_DECODER_IMPL_H

#include <rfid/tag_decoder.h>
#include <vector>
#include "rfid/global_vars.h"
#include <time.h>
#include <numeric>
#include <fstream>

#define DEBUG_DECODER_RN16
#define DEBUG_DECODER_EPC

namespace gr
{
  namespace rfid
  {
    class tag_decoder_impl : public tag_decoder
    {
      private:
        float n_samples_TAG_BIT;
        int s_rate;
        char * char_bits;

        class sample_information
        {
          private:
            gr_complex* _in;
            int _total_size;
            std::vector<float> _norm_in;

            float _corr;

          public:
            sample_information();
            sample_information(gr_complex*, int);
            ~sample_information();

            void set_corr(float);

            gr_complex in(int);
            int total_size(void);
            float norm_in(int);

            float corr(void);
        };

        // tag_decoder_impl.cc
        int check_crc(char*, int);

        // tag_decoder_decoder.cc
        int tag_sync(sample_information*);
        std::vector<float> tag_detection(sample_information*, int, int);
        int determine_first_mask_level(sample_information*, int);
        int decode_single_bit(sample_information* in, int, int);

        // debug_message
        std::string current_round_slot;
        std::ofstream debug_log;
        #ifdef DEBUG_DECODER_RN16
        std::ofstream debug_decoder_RN16_i;
        std::ofstream debug_decoder_RN16_q;
        std::ofstream debug_decoder_RN16;
        #endif
        #ifdef DEBUG_DECODER_EPC
        std::ofstream debug_decoder_EPC_i;
        std::ofstream debug_decoder_EPC_q;
        std::ofstream debug_decoder_EPC;
        #endif

      public:
        tag_decoder_impl(int, std::vector<int>);
        ~tag_decoder_impl();
        void forecast (int, gr_vector_int&);
        int general_work(int, gr_vector_int&, gr_vector_const_void_star&, gr_vector_void_star&);
    };
  }
}

#endif
