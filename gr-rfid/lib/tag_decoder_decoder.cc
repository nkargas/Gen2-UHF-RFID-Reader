/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "tag_decoder_impl.h"

#define SHIFT_SIZE 2  // used in tag_detection

//#define __DEBUG__

namespace gr
{
  namespace rfid
  {
    int tag_decoder_impl::tag_sync(sample_information* ys, int n_expected_bit)
      // This method searches the preamble and returns the start index of the tag data.
      // If the correlation value exceeds the threshold, it returns the start index of the tag data.
      // Else, it returns -1.
      // Threshold is an experimental value, so you might change this value within your environment.
    {
      int win_size = n_samples_TAG_BIT * TAG_PREAMBLE_BITS;
      float threshold = n_samples_TAG_BIT * 4;  // threshold verifing correlation value

      float max_corr = 0.0f;
      int max_index = 0;

      // compare all samples with sliding
      for(int i=0 ; i<ys->total_size()-(n_samples_TAG_BIT*(TAG_PREAMBLE_BITS+n_expected_bit)) ; i++)  // i: start point
      {
        // calculate average_amp (threshold)
        float average_amp = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          average_amp += ys->norm_in(i+j);
        average_amp /= win_size;

        // calculate normalize_factor
        float standard_deviation = 0.0f;
        for(int j=0 ; j<win_size ; j++)
          standard_deviation += pow(ys->norm_in(i+j) - average_amp, 2);
        standard_deviation /= win_size;
        standard_deviation = sqrt(standard_deviation);

        // calculate correlation value
        float corr_candidates[2] = {0.0f};
        for(int j=0 ; j<2*TAG_PREAMBLE_BITS ; j++)  // j: half_bit index of TAG_PREAMBLE
        {
          for(int k=0 ; k<(n_samples_TAG_BIT/2.0) ; k++)
          {
            for(int m=0 ; m<2 ; m++)  // m: index of TAG_PREAMBLE type
              corr_candidates[m] += TAG_PREAMBLE[m][j] * ((ys->norm_in(i + j*(int)(n_samples_TAG_BIT/2.0) + k) - average_amp) / standard_deviation);
          }
        }

        // get max correlation value for ith start point
        float corr = 0.0f;
        for(int j=0 ; j<2 ; j++)
          if(corr_candidates[j] > corr) corr = corr_candidates[j];

        // compare with current max correlation value
        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
        }
      }

#ifdef __DEBUG__
      debug_log << "threshold= " << threshold << std::endl;
      debug_log << "corr= " << max_corr << std::endl;
      debug_log << "preamble index= " << max_index << std::endl;
      debug_log << "sample index= " << max_index + win_size << std::endl;
#endif


      // check if correlation value exceeds threshold
      if(max_corr > threshold) return max_index + win_size;
      else return -1;
    }

    std::vector<float> tag_decoder_impl::tag_detection(sample_information* ys, int index, int n_expected_bit)
      // This method decodes n_expected_bit of data by using previous methods, and returns the vector of the decoded data.
      // index: start point of "data bit", do not decrease half bit!
    {
      std::vector<float> decoded_bits;

      int mask_level = determine_first_mask_level(ys, index);
      int complex_mask_level = -1;
      int shift = 0;
      double max_corr_sum = 0.0f;
      gr_complex max_complex_corr_sum(0.0, 0.0);

      for(int i=0 ; i<n_expected_bit ; i++)
      {
        int idx = index + i*n_samples_TAG_BIT + shift;  // start point of decoding bit with shifting
        float max_corr = 0.0f;
        gr_complex max_complex_corr(0.0, 0.0);
        int max_index;
        int curr_shift;

        // shifting from idx-SHIFT_SIZE to idx+SHIFT_SIZE
        for(int j=0 ; j<(SHIFT_SIZE*2 + 1) ; j++)
        {
          int index = decode_single_bit(ys, idx+j-SHIFT_SIZE, mask_level, complex_mask_level);
          float corr = ys->corr();

          if(corr > max_corr)
          {
            max_corr = corr;
            max_index = index;
            max_complex_corr = ys->complex_corr();
            curr_shift = j - SHIFT_SIZE;  // find the best current shift value
          }
        }

        max_corr_sum += max_corr;
        max_complex_corr_sum += max_complex_corr;

        
#ifdef __DEBUG__
        debug_log << "[" << i+1 << "th bit]\tcorr=";
        debug_log << std::left << std::setw(8) << max_corr;
        debug_log << "\tcurr_shift=" << curr_shift << "\tshift=";
        debug_log << std::left << std::setw(5) << shift;
        debug_log << "\tdecoded_bit=" << max_index;
        if(mask_level) debug_log << " (high start)" << std::endl;
        else debug_log << " (low start)" << std::endl;
#endif

        if(max_index){
          mask_level *= -1; // change mask_level(start level of the next bit) when the decoded bit is 1
          complex_mask_level *= -1;
        }

        decoded_bits.push_back(max_index);
        shift += curr_shift;  // update the shift value
      }

      ys->set_corr(max_corr_sum/n_expected_bit);
      ys->set_complex_corr(max_complex_corr_sum/(float)n_expected_bit);

      return decoded_bits;
    }

    int tag_decoder_impl::determine_first_mask_level(sample_information* ys, int index)
      // This method searches whether the first bit starts with low level or high level.
      // If the first bit starts with low level, it returns -1.
      // If the first bit starts with high level, it returns 0.
      // index: start point of "data bit", do not decrease half bit!
    {
      decode_single_bit(ys, index, -1, -1);
      float low_level_corr = ys->corr();

      decode_single_bit(ys, index, 1, 1);
      if(low_level_corr > ys->corr()) return -1;
      else return 1;
    }

    int tag_decoder_impl::decode_single_bit(sample_information* ys, int index, int mask_level, int complex_mask_level)
      // This method decodes single bit and returns the decoded value and the correlation score.
      // index: start point of "data bit", do not decrease half bit!
      // mask_level: start level of "decoding bit". (-1)low level start, (1)high level start.
    {
      const float masks[2][2][4] = { // first, last elements are extra bits. second, third elements are real signal.
        {{1, -1, 1, -1}, {1, -1, -1, 1}}, // low level start
        {{-1, 1, -1, 1}, {-1, 1, 1, -1}}  // high level start
      };

      if(mask_level == -1) mask_level = 0;  // convert for indexing
      if(complex_mask_level == -1) complex_mask_level = 0;  // convert for indexing


      float max_corr = 0.0f;
      gr_complex max_complex_corr(0.0, 0.0);
      int max_index = -1;

      float average_amp = 0.0f;
      gr_complex average_complex_amp = std::complex<float>(0.0,0.0);
      for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++){
        average_amp += ys->norm_in(index+j);
        average_complex_amp += ys->in(index+j);
      }
      average_amp /= (2*n_samples_TAG_BIT);
      average_complex_amp /= (2*n_samples_TAG_BIT);

      // compare with two masks (0 or 1)
      for(int i=0 ; i<2 ; i++)
      {
        float corr = 0.0f;
        gr_complex complex_corr(0.0,0.0);
        for(int j=-(n_samples_TAG_BIT*0.5) ; j<(n_samples_TAG_BIT*1.5) ; j++)
        {
          int idx;
          if(j < 0) idx = 0;                            // first section (trailing half bit of the previous bit)
          else if(j < (n_samples_TAG_BIT*0.5)) idx = 1; // second section (leading half bit of the data bit)
          else if(j < n_samples_TAG_BIT) idx = 2;       // third section (trailing half bit of the data bit)
          else idx = 3;                                 // forth section (leading half bit of the later bit)

          //corr += masks[mask_level][i][idx] * std::sqrt(std::norm((ys->in(index+j) - average_complex_amp)));
          corr += masks[mask_level][i][idx] * (ys->norm_in(index+j) - average_amp);
          complex_corr += (masks[complex_mask_level][i][idx] * (ys->in(index+j) - average_complex_amp));
        }
        //corr = std::abs(corr);

        if(corr > max_corr)
        {
          max_corr = corr;
          max_index = i;
          max_complex_corr = complex_corr;
        }
      }

      ys->set_corr(max_corr/(2*n_samples_TAG_BIT));
      ys->set_complex_corr(max_complex_corr/(2*n_samples_TAG_BIT));
      return max_index;
    }
  }
}
