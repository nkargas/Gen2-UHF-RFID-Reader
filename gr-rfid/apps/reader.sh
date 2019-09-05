#! /bin/sh
rm -r debug_data
mkdir debug_data

cd debug_data
mkdir log
mkdir RN16_input EPC_input
mkdir RN16_preamble EPC_preamble
mkdir RN16_sample EPC_sample
mkdir gate
cd ../

rm log result
python reader.py
cat result
