#!/bin/bash
ODIR="configs"

run_onecase() {
    for i in $(seq 0 0); do
        TRACE_PATH="../../cqi-traces-noise0"
        cp $TRACE_PATH/mapping$i.config $TRACE_PATH/mapping.config

        ../../LTE-Sim SingleCellWithI 1 100 1 30 $i 12 ${ODIR}/config.json 2> ${ODIR}/ours_${i}.log > /dev/null &
        # ../../LTE-Sim SingleCellWithI 1 200 1 30 $i 12 ${ODIR}/config.json 2> ${ODIR}/ours_maxcell_${i}.log > /dev/null &
        # ../../LTE-Sim SingleCellWithI 1 7 1 30 $i 12 ${ODIR}/config.json 2> ${ODIR}/nvs_${i}.log > /dev/null &
        ../../LTE-Sim SingleCellWithI 1 9 1 30 $i 12 ${ODIR}/config.json 2> ${ODIR}/maxcell_${i}.log > /dev/null &
        ../../LTE-Sim SingleCellWithI 1 95 1 30 $i 12 ${ODIR}/config.json 2> ${ODIR}/maxcell_capped_${i}.log > /dev/null &
        ../../LTE-Sim SingleCellWithI 1 102 1 30 $i 12 ${ODIR}/config.json 2> ${ODIR}/greedy_edf_${i}.log > /dev/null
        sleep 1
    done
}

run_onecase