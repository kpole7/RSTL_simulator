#!/bin/bash

for i in {1..16}; do
    # Formatowanie numeru seryjnego z wiodącym zerem dla wartości <10
    serial_num=$(printf "97J-%04d" $i)
    
    gnome-terminal --title="RSTL Simulator $i" -- bash -c \
        "cd ~/LocalSoftware/RSTL_simulator && \
         ./RSTL_simulator /tmp/server$i \"Rev 3.05 RSTL 15-300 Serial $serial_num\"; \
         exec bash"
done
