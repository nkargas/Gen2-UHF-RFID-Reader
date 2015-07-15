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
#include "rfid/global_vars.h"

#include <iostream>
namespace gr {
  namespace rfid {
    
    READER_STATE * reader_state;

    void initialize_reader_state()
    {
      reader_state = new READER_STATE;
      reader_state-> reader_stats.n_queries_sent = 0;
      reader_state-> reader_stats.n_epc_correct = 0;

      std::vector<int>  unique_tags_round;
       std::map<int,int> tag_reads;    

      reader_state-> status           = RUNNING;
      reader_state-> gen2_logic_status= START;
      reader_state-> gate_status       = GATE_SEEK_RN16;
      reader_state-> decoder_status   = DECODER_DECODE_RN16;

      reader_state-> reader_stats.max_slot_number = pow(2,FIXED_Q);

      reader_state-> reader_stats.cur_inventory_round = 1;
      reader_state-> reader_stats.cur_slot_number     = 1;

      gettimeofday (&reader_state-> reader_stats.start, NULL);
    }
  } /* namespace rfid */
} /* namespace gr */

