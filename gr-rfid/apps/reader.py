#Developed by: Nikos Kargas 

from gnuradio import gr
from gnuradio import uhd
from gnuradio import blocks
from gnuradio import filter
from gnuradio import analog
from gnuradio import digital
from gnuradio import qtgui
import rfid

DEBUG = False

class reader_top_block(gr.top_block):

  # Configure usrp source
  def u_source(self):
    self.source = uhd.usrp_source(
    device_addr=self.usrp_address_source,
    stream_args=uhd.stream_args(
    cpu_format="fc32",
    channels=range(1),
    ),
    )
    self.source.set_samp_rate(self.adc_rate)
    self.source.set_center_freq(self.freq, 0)
    self.source.set_gain(self.rx_gain, 0)
    self.source.set_antenna("RX2", 0)
    #self.source.set_auto_dc_offset(False) # Uncomment this line for SBX daughterboard

  # Configure usrp sink
  def u_sink(self):
    self.sink = uhd.usrp_sink(
    device_addr=self.usrp_address_sink,
    stream_args=uhd.stream_args(
    cpu_format="fc32",
    channels=range(1),
    ),
    )
    self.sink.set_samp_rate(self.dac_rate)
    self.sink.set_center_freq(self.freq, 0)
    self.sink.set_gain(self.tx_gain, 0)
    self.sink.set_antenna("TX/RX", 0)
    
  def __init__(self):
    gr.top_block.__init__(self)


    #rt = gr.enable_realtime_scheduling() 

    ######## Variables #########
    self.dac_rate = 1e6                 # DAC rate 
    self.adc_rate = 100e6/50            # ADC rate (2MS/s complex samples)
    self.decim     = 5                    # Decimation (downsampling factor)
    self.ampl     = 0.1                  # Output signal amplitude (signal power vary for different RFX900 cards)
    self.freq     = 910e6                # Modulation frequency (can be set between 902-920)
    self.rx_gain   = 20                   # RX Gain (gain at receiver)
    self.tx_gain   = 0                    # RFX900 no Tx gain option

    self.usrp_address_source = "addr=192.168.10.2,recv_frame_size=256"
    self.usrp_address_sink   = "addr=192.168.10.2,recv_frame_size=256"

    # Each FM0 symbol consists of ADC_RATE/BLF samples (2e6/40e3 = 50 samples)
    # 10 samples per symbol after matched filtering and decimation
    self.num_taps     = [1] * 25 # matched to half symbol period

    ######## File sinks for debugging (1 for each block) #########
    self.file_sink_source         = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/source", False)
    self.file_sink_matched_filter = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/matched_filter", False)
    self.file_sink_gate           = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/gate", False)
    self.file_sink_decoder        = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/decoder", False)
    self.file_sink_reader         = blocks.file_sink(gr.sizeof_float*1,      "../misc/data/reader", False)

    ######## Blocks #########
    self.matched_filter = filter.fir_filter_ccc(self.decim, self.num_taps);
    self.gate            = rfid.gate(int(self.adc_rate/self.decim))
    self.tag_decoder    = rfid.tag_decoder(int(self.adc_rate/self.decim))
    self.reader          = rfid.reader(int(self.adc_rate/self.decim),int(self.dac_rate))
    self.amp              = blocks.multiply_const_ff(self.ampl)
    self.to_complex      = blocks.float_to_complex()

    if (DEBUG == False) : # Real Time Execution

      # USRP blocks
      self.u_source()
      self.u_sink()

      ######## Connections #########
      self.connect(self.source,  self.matched_filter)
      self.connect(self.matched_filter, self.gate)

      self.connect(self.gate, self.tag_decoder)
      self.connect((self.tag_decoder,0), self.reader)
      self.connect(self.reader, self.amp)
      self.connect(self.amp, self.to_complex)
      self.connect(self.to_complex, self.sink)

      #File sinks for logging (Remove comments to log data)
      #self.connect(self.source, self.file_sink_source)

    else :  # Offline Data
      self.file_source               = blocks.file_source(gr.sizeof_gr_complex*1, "../misc/data/file_source_test",False)   ## instead of uhd.usrp_source
      self.file_sink                  = blocks.file_sink(gr.sizeof_gr_complex*1,   "../misc/data/file_sink", False)     ## instead of uhd.usrp_sink
 
      ######## Connections ######### 
      self.connect(self.file_source, self.matched_filter)
      self.connect(self.matched_filter, self.gate)
      self.connect(self.gate, self.tag_decoder)
      self.connect((self.tag_decoder,0), self.reader)
      self.connect(self.reader, self.amp)
      self.connect(self.amp, self.to_complex)
      self.connect(self.to_complex, self.file_sink)
    
    #File sinks for logging 
    #self.connect(self.gate, self.file_sink_gate)
    self.connect((self.tag_decoder,1), self.file_sink_decoder) # (Do not comment this line)
    #self.connect(self.file_sink_reader, self.file_sink_reader)
    #self.connect(self.matched_filter, self.file_sink_matched_filter)

if __name__ == '__main__':

  main_block = reader_top_block()
  main_block.start()

  while(1):
    inp = raw_input("'Q' to quit \n")
    if (inp == "q" or inp == "Q"):
      break

  main_block.reader.print_results()
  main_block.stop()
