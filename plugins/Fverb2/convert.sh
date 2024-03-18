#!/bin/bash
## install faust
# wget https://github.com/grame-cncm/faust/releases/download/2.70.3/faust-2.70.3.tar.gz
# tar -xvzf faust-*.tar.gz
# cd faust-*
# make
# sudo make install

## convert fverb dsp to sc
faust2sc.py -p ~/Documents/supercollider fverb2.dsp


