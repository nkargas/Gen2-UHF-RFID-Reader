#! /bin/sh
rm -r debug_data
mkdir debug_data
cd debug_data
mkdir RN16_iq
cd RN16_iq
mkdir graph
cd ../../
rm debug_message result
python reader.py
cat result
rm a
