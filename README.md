# Gen2 UHF RFID Reader
This is a Gen2 UHF RFID Reader. It is able to identify commercial Gen2 RFID Tags with FM0 line coding and 40kHz data rate (BLF), and extract their EPC message. It requires USRPN200 and was tested with a single front-end card (RFX900).  
Developed by Nikos Kargas in the context of his MSc thesis, School of Electronic & Computer Engineering, Technical Univ. of Crete.  

The project was initially based on the RFID Gen2 Reader available at https://github.com/ransford/gen2_rfid (previously at https://www.cgran.org/wiki/Gen2 Contributor: Michael Buettner). In order to make it compatible with the latest version of GNU Radio, UHD as well as USRPN200, I decided to write a Gen2 Reader from scratch. The reader borrows elements from the software developed by Buettner, i.e. Data flow: Gate -> Decoder -> Reader as well as the conception regarding the detection of the reader commands. CRC calculation and checking functions were also adapted from https://www.cgran.org/browser/projects/gen2_rfid/.

### Implemented GNU Radio Blocks:

- Gate : Responsible for reader command detection.  
- Tag decoder : Responsible for frame synchronization, channel estimation, symbol period estimation and detection.  
- Reader (Gen2 Logic) : Create/send reader commands.

## Installation

- install UHD driver + GNU Radio using **wget http://www.sbrac.org/files/build-gnuradio && chmod a+x ./build-gnuradio && ./build-gnuradio**
- cd Gen2-UHF-RFID-Reader/gr-rfid/  
- mkdir build  
- cd build/  
- cmake ../ (logging should be enabled)  
- sudo make install  

## Configuration

- Set USRPN200 address in apps/reader.py (default: 192.168.10.2)
- Set frequency in apps/reader.py (default: 910MHz)
- Set tx amplitude in apps/reader.py (default: 0.1)
- Set rx gain in apps/reader.py (default: 20)
- Set maximum number of queries in include/global_vars.h (default:1000)
- Set number of inventory round slots in include/global_vars.h (default: 0)

## How to run

- Real time execution:  
cd Gen2-UHF-RFID-Reader/gr-rfid/apps/    
sudo GR_SCHEDULER=STS nice -n -20 python ./reader.py     
After termination, part of EPC message (hex value of EPC[104:111]) of identified Tags is printed.  

- Offline:  
    Change DEBUG variable in apps/reader.py to TRUE (A test file already exists named file_source_test).  
    The reader works with offline traces without using a USRP.  
    The output after running the software with test file is:  
    
    | Number of queries/queryreps sent : 71  
    | Current Inventory round : 72  

    | Correctly decoded EPC : 70  
    | Number of unique tags : 1  
    | Tag ID : 27  Num of reads : 70  


    #### Output files 
    
    /misc/data/source  
    /misc/data/matched_filter  
    /misc/data/gate 
    /misc/data/decoder  
    /misc/data/reader
    
    #### Plot using Octave/MATLAB
    /misc/code/plot_signal.m       
    
## Logging

- Configuration file : /home/username/.gnuradio/config.conf  
    Edit the above file and add the following lines  

    [LOG]  
    debug_file = /PathToLogFile/Filename  
    debug_level = info  
    
    Logging may cause latency issues if it is enabled during real time execution. However, it can be used with offline traces.
    
## Hardware:

  - 1x USRPN200/N210  
  - 1x RFX900 daughterboard  
  - 2x circular polarized antennas  
  
### Update:

To run with a SBX daugherboard, uncomment the line #self.source.set_auto_dc_offset(False) in reader.py file.
  
## Tested on:
  Ubuntu 14.04 64-bit  
  GNU Radio 3.7.4
  
## For more information:
N. Kargas, F. Mavromatis and A. Bletsas, "Fully-Coherent Reader with Commodity SDR for Gen2 FM0 and Computational RFID", accepted for publication, IEEE Wireless Communications Letters (WCL), to appear in Fall 2015.

## Contact:
  Nikos Kargas (e-mail1: cpznick@gmail.com email2: karga005@umn.edu)  

This research has been co-financed by the European Union (European Social Fund-ESF) and Greek national funds through the Operational Program Education and Lifelong Learning of the National Strategic Reference Framework (NSRF) - Research Funding Program: THALES-Investing in knowledge society through the European Social Fund.