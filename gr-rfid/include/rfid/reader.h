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

#ifndef INCLUDED_RFID_READER_H
#define INCLUDED_RFID_READER_H

#include <rfid/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace rfid {

    /*!
     * \brief The block is responsible for sending commands for transmission.
     *
     * It moves between the following states.
     *
     * \ingroup rfid
     *
     */
    class RFID_API reader : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<reader> sptr;
      virtual void print_results() =0;
      /*!
       * \brief Return a shared_ptr to a new instance of rfid::reader.
       *
       * To avoid accidental use of raw pointers, rfid::reader's
       * constructor is in a private implementation
       * class. rfid::reader::make is the public interface for
       * creating new instances.
       */
      static sptr make(int sample_rate, int dac_rate);

    };

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_READER_H */

