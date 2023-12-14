#!/usr/bin/python3
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
from collections import defaultdict
import numpy as np

BEGIN_IDX=2
INTRA=""
INPUT_DIR="three_video_15slices"
FTYPE=".png"
COLORS=["brown", "dimgrey", "cornflowerblue"]

def get_cdf(data, ratio=0, upperbound=100000):
    data.sort()
    
    x_array = []
    y_array = []
    for i, item in enumerate(data):
        if i / len(data) >= ratio:
            if item > upperbound:
                break
            x_array.append( item )
            y_array.append( i / len(data) )
    # return x_array, y_array
    # Calculate the 99th percentile
    index_99_percentile = int(0.99 * len(data)) - 1
    percentile_99 = data[index_99_percentile] if index_99_percentile < len(data) else None

    return x_array, y_array, percentile_99

def get_cdf_tail(data, ratio=0, upperbound=100000, tail = 0.99):
    data.sort()
    
    x_array = []
    y_array = []
    tail_latency = -1
    for i, item in enumerate(data):
        cdf_value = (i + 1) / len(data)
        if i / len(data) >= ratio:
            if item > upperbound:
                break
            x_array.append( item )
            y_array.append( i / len(data) )
            
            if cdf_value >= tail and tail_latency == -1:
                tail_latency = item
                break
    return x_array, y_array, tail_latency

def get_hol(fname, slice_begin, slice_end):
    hol_array = []
    with open(fname, "r") as fin:
        for line in fin:
            words = line.split(" ")
            if not words[0].isdigit():
                continue
            flow = int(words[2])
            sid = int(words[12])
            if sid >= slice_begin and sid <= slice_end:
                hol_array.append( float(words[8] ) )
    return hol_array

def get_fct(fname, slice_begin, slice_end, priority_only):
    flow_to_slice = {}
    fct_captured = {}
    # only analyze flows generated before ts_shoot
    ts_shoot = 10000
    is_after_ts_shoot = False
    fct_array = []
    with open(fname, "r") as fin:
        for line in fin:
            words = line.split(" ")
            if words[0].isdigit():
                app_id = int(words[2])
                flow_to_slice[app_id] = int(words[12])
    with open(fname, "r") as fin:
        for line in fin:
            words = line.split(" ")
            if words[0].isdigit():
                if int(words[0]) > ts_shoot:
                    is_after_ts_shoot = True
            if words[0] == "ipflow":
                # even number is higher priority
                app_id = int(words[3])
                # skip flows not belong to this slice
                if app_id in flow_to_slice:
                    if flow_to_slice[app_id] < slice_begin or flow_to_slice[app_id] > slice_end:
                        continue
                else:
                    print("%s, flow: %d never scheduled" % (fname, app_id) )

                if words[1] == "start" and not is_after_ts_shoot:
                    key =( app_id, int(words[5]) )
                    fct_captured[key] = -1
                if words[1] == "end":
                    # skip non-priority flows
                    if priority_only and int(words[11]) == 0:
                        continue
                    key =( app_id, int(words[5]) )
                    if key in fct_captured:
                        fct_captured[key] = float( words[7] )
    for k, v in fct_captured.items():
        if v != -1:
            fct_array.append( v )
    print( "%s: finished %d flows before %dms" % ( fname, len(fct_array), ts_shoot) )
    return fct_array

def get_throughput(fname, slice_begin, slice_end):
    perflow_throughput = {}
    cumu_throughput = {}
    begin_ts = 20000
    end_ts = 22000
    for i in range(slice_begin, slice_end):
        perflow_throughput[i] = {}
        cumu_throughput[i] = 0
    with open(fname, "r") as fin:
        for line in fin:
            words = line.split(" ")
            if not words[0].isdigit():
                continue
            if int(words[0]) > end_ts:
                break
            if int(words[0]) > begin_ts:
                flow = int(words[2])
                sid = int(words[12])
                if sid >= slice_begin and sid < slice_end:
                    perflow_throughput[sid][flow] = int( words[4] ) / (end_ts / 1000 ) * 8 / (1000 * 1000)
    for i in range(slice_begin, slice_end):
        for k in perflow_throughput[i].keys():
            cumu_throughput[i] += perflow_throughput[i][k]
    return [v for v in cumu_throughput.values()]

def get_fct_schemes(dname, slice_begin, slice_end, priority_only):
    all_fct = {}
    for i in range(BEGIN_IDX, BEGIN_IDX+1):
        all_fct['mt'] = get_fct( dname + "/max_throughput_" + str(i) + ".log", slice_begin, slice_end, priority_only )
        all_fct['mlwdf'] = get_fct( dname + "/mlwdf_" + str(i) + ".log", slice_begin, slice_end, priority_only )
        all_fct['pf'] = get_fct( dname + "/pf_" + INTRA + str(i) + ".log", slice_begin, slice_end, priority_only )
    return all_fct

def get_hol_schemes(dname, slice_begin, slice_end, begin_idx):
    all_hol = {}
    for i in range(begin_idx, begin_idx+1):
        all_hol['mt'] = get_hol( dname + "/max_throughput_" + str(i) + ".log", slice_begin, slice_end)
        all_hol['mlwdf'] = get_hol( dname + "/mlwdf_" + str(i) + ".log", slice_begin, slice_end )
        all_hol['pf'] = get_hol( dname + "/pf_" + INTRA + str(i) + ".log", slice_begin, slice_end )
        # print("mt: ", all_hol["mt"])
        # print("mlwdf: ", all_hol["mlwdf"])
        # print("pf: ", all_hol["pf"])
        
    return all_hol

def get_throughput_schemes(dname, slice_begin, slice_end):
    all_throughput = {}
    for i in range(BEGIN_IDX, BEGIN_IDX+1):
        all_throughput['mt'] = np.mean(
                get_throughput( dname + "/max_throughput_" + str(i) + ".log", slice_begin, slice_end ) )
        all_throughput['mlwdf'] = np.mean(
                get_throughput( dname + "/mlwdf_" + str(i) + ".log", slice_begin, slice_end ) )
        all_throughput['pf'] = np.mean(
                get_throughput( dname + "/pf_" + INTRA + str(i) + ".log", slice_begin, slice_end ) )
    return all_throughput

def print_throughput(slice_begin, slice_end):
    all_throughput = get_throughput_schemes(
            INPUT_DIR, slice_begin, slice_end )
    print(all_throughput)

def plot_fct(ofname, slice_begin, slice_end, priority_only=False):
    default_font = 20
    fig, ax = plt.subplots(figsize=(9, 5))
    all_fct = get_fct_schemes(INPUT_DIR, slice_begin, slice_end, priority_only)
    mt_x, mt_y = get_cdf( all_fct['mt'], 0.0, 5 )
    mlwdf_x, mlwdf_y = get_cdf( all_fct['mlwdf'], 0.0, 5 )
    pf_x, pf_y = get_cdf( all_fct['pf'], 0.0, 5 )
    # mt_x, mt_y = get_cdf( all_fct['mt'], 0.0 )
    # mlwdf_x, mlwdf_y = get_cdf( all_fct['mlwdf'], 0.0 )
    # pf_x, pf_y = get_cdf( all_fct['pf'], 0.0 )

    # print(ofname)
    # print("mt: %f %f %f" % ( np.mean( mt_x ), mt_x[int( len(mt_x)*0.5 )], mt_x[int( len(mt_x)*0.95 )] ) )
    # print("mlwdf: %f %f %f" % ( np.mean( mlwdf_x ), mlwdf_x[int( len(mlwdf_x)*0.5 )], mlwdf_x[int( len(mlwdf_x)*0.95 )] ) )
    # print("pf: %f %f %f" % ( np.mean( pf_x ), pf_x[int( len(pf_x)*0.5 )], pf_x[int( len(pf_x)*0.95 )] ) )
    # return
    
    ax.plot( mlwdf_x, mlwdf_y, "--", label="mlwdf", color=COLORS[2], linewidth=2.5 )
    ax.plot( pf_x, pf_y, "--", label="pf", color=COLORS[0], linewidth=2.5 )
    ax.plot( mt_x, mt_y, "--", label="mt", color=COLORS[1], linewidth=2.5 )
    ax.set_xlabel("Flow Completion Time(s)", fontsize=default_font + 4)
    ax.set_ylabel("Ratio", fontsize=default_font + 4)
    ax.legend(fontsize=default_font + 2)
    ax.tick_params(axis="both", labelsize=default_font)
    ax.grid( axis="both", alpha=0.2 )
    plt.tight_layout()
    fig.savefig( ofname + FTYPE )

def plot_hol_delay(ofname, slice_begin, slice_end, plot_info, begin_idx):
    default_font = 20
    fig, ax = plt.subplots(figsize=(9, 5))
    all_hol = get_hol_schemes( INPUT_DIR, slice_begin, slice_end, begin_idx)
    mt_x, mt_y = get_cdf( all_hol['mt'], 0.0, 10 )
    mlwdf_x, mlwdf_y = get_cdf( all_hol['mlwdf'], 0.0, 10 )
    pf_x, pf_y = get_cdf( all_hol['pf'], 0.0, 10 )

    ax.plot( pf_x, pf_y, "y--", label="pf", color=COLORS[0], linewidth=2.5 )
    ax.plot( mt_x, mt_y, "r--", label="mt", color=COLORS[1], linewidth=2.5 )
    ax.plot( mlwdf_x, mlwdf_y, "b--", label="mlwdf", color=COLORS[2], linewidth=2.5 )
    ax.set_xlabel("Queueing Delay(s)", fontsize=default_font + 4)
    ax.set_ylabel("Ratio", fontsize=default_font + 4)
    ax.legend(fontsize=default_font + 2)
    ax.tick_params(axis="both", labelsize=default_font)
    ax.grid( axis="both", alpha=0.2 )
    plt.tight_layout()
    fig.savefig( ofname + plot_info + FTYPE )
    
    
def plot_hol_delay_compare_15_slices(begin_idx):
    default_font = 20
    # fig, ax = plt.subplots(figsize=(9, 5))
    all_hol_3 = get_hol_schemes( INPUT_DIR, 10, 14, begin_idx)
    mt_x_3, mt_y_3, mt_99_3 = get_cdf( all_hol_3['mt'], 0.0, 30 )
    mlwdf_x_3, mlwdf_y_3, mlwdf_99_3= get_cdf( all_hol_3['mlwdf'], 0.0, 30 )
    pf_x_3, pf_y_3, pf_99_3 = get_cdf( all_hol_3['pf'], 0.0, 30 )
    
    all_hol_2 = get_hol_schemes( INPUT_DIR, 5, 9, begin_idx)
    mt_x_2, mt_y_2, mt_99_2= get_cdf( all_hol_2['mt'], 0.0, 30 )
    mlwdf_x_2, mlwdf_y_2, mlwdf_99_2= get_cdf( all_hol_2['mlwdf'], 0.0, 30 )
    pf_x_2, pf_y_2, pf_99_2 = get_cdf( all_hol_2['pf'], 0.0, 30 )

   
    all_hol_1 = get_hol_schemes( INPUT_DIR, 0, 4, begin_idx)
    mt_x_1, mt_y_1, mt_99_1 = get_cdf( all_hol_1['mt'], 0.0, 30 )
    mlwdf_x_1, mlwdf_y_1, mlwdf_99_1= get_cdf( all_hol_1['mlwdf'], 0.0, 30 )
    pf_x_1, pf_y_1, pf_99_1= get_cdf( all_hol_1['pf'], 0.0, 30 )
    
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
    
    ax1.plot( mlwdf_x_1, mlwdf_y_1, "y--", label="slice 0 till 4", color=COLORS[0], linewidth=2.5 )
    ax1.plot( mlwdf_x_3, mlwdf_y_3, "b--", label="slice 10 till 14", color=COLORS[2], linewidth=2.5 )
    ax1.plot( mlwdf_x_2, mlwdf_y_2, "r--", label="slice 5 till 9", color=COLORS[1], linewidth=2.5 )
    ax1.set_xlabel("Queueing Delay(s)", fontsize=default_font + 4)
    ax1.set_ylabel("Ratio", fontsize=default_font + 4)
    ax1.legend(fontsize=default_font + 2)
    ax1.tick_params(axis="both", labelsize=default_font)
    ax1.title.set_text("MLWDF")
    ax1.grid( axis="both", alpha=0.2 )
    
    
    ax2.plot( mt_x_1, mt_y_1, "y--", label="slice 0 till 4", color=COLORS[0], linewidth=2.5 )
    ax2.plot( mt_x_3, mt_y_3, "b--", label="slice 10 till 14", color=COLORS[2], linewidth=2.5 )
    ax2.plot( mt_x_2, mt_y_2, "r--", label="slice 5 till 9", color=COLORS[1], linewidth=2.5 )
    
    ax2.set_xlabel("Queueing Delay(s)", fontsize=default_font + 4)
    ax2.set_ylabel("Ratio", fontsize=default_font + 4)
    ax2.legend(fontsize=default_font + 2)
    ax2.tick_params(axis="both", labelsize=default_font)
    ax2.grid( axis="both", alpha=0.2 )
    ax2.title.set_text("Max Throughput")
    plt.tight_layout()
    # plt.title("the seed is now: " + str(begin_idx))
    fig.savefig( "mlwdf_vs_mt" + str(begin_idx) + FTYPE )
    
    print("mt_99_1: %f, mt_99_2: %f, mt_99_3: %f" % (mt_99_1, mt_99_2, mt_99_3))
    print("mlwdf_99_1: %f, mlwdf_99_2: %f, mlwdf_99_3: %f" % (mlwdf_99_1, mlwdf_99_2, mlwdf_99_3))
    print("pf_99_1: %f, pf_99_2: %f, pf_99_3: %f" % (pf_99_1, pf_99_2, pf_99_3))
    print("******************************")
    

print_throughput( 2, 5)
# plot_fct( "cdf-fct-pf", 5, 9 )
# plot_hol_delay( "cdf-hol-delay", 0,4, "slice 0 to 4")
plot_hol_delay_compare_15_slices(0)
plot_hol_delay_compare_15_slices(1)
plot_hol_delay_compare_15_slices(2)

# plot_hol_delay( "cdf-hol-delay", 5, 9 , "slice 5 to 9")
# plot_hol_delay( "cdf-hol-delay", 10, 14 , "slice 10 to 14")