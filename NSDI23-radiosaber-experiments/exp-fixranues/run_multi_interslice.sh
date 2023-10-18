#!/bin/bash
ODIR="slices"

sliceArray=("5" "10" "15" "20")

run_onecase() {
    i=$1
    for n in ${sliceArray[@]}; do
        ../../LTE-Sim SingleCellWithI 1 7 1 30 $i 12 ${n}${ODIR}/config-${2}.json 2> ${n}${ODIR}/nvs_${2}$i.log > /dev/null &
        ../../LTE-Sim SingleCellWithI 1 8 1 30 $i 12 ${n}${ODIR}/config-${2}.json 2> ${n}${ODIR}/sequential_${2}$i.log > /dev/null &
        ../../LTE-Sim SingleCellWithI 1 9 1 30 $i 12 ${n}${ODIR}/config-${2}.json 2> ${n}${ODIR}/maxcell_${2}$i.log > /dev/null &
        ../../LTE-Sim SingleCellWithI 1 10 1 30 $i 12 ${n}${ODIR}/config-${2}.json 2> ${n}${ODIR}/upperbound_${2}$i.log > /dev/null &
    done
}

TRACE_PATH="$HOME/Research/RadioSaber/cqi-traces-noise0/"
cp $TRACE_PATH/mapping0.config $TRACE_PATH/mapping.config
run_onecase 0 mt
