# Plot the throughput of UE compared with its GBR
# x: time
# y: throughput

import os
import matplotlib.pyplot as plt
import numpy as np


folder = "configs"
filename = "nvs_0.log"
#filename = "maxcell_0.log"
#filename = "single_0.log"
scheme = "nvs"
#scheme = "Ours"
n_users = 100
# 1s = 1000ms
period = 1000 # throughput measurement window size: number of TTIs
# 1000 TTIs = 1s
begin_ts = 100
end_ts = 10000 + begin_ts # end time of the measurement
print("end_ts:", end_ts)
flag  = {}
for i in range(n_users):
    ue_flag = {}
    for j in range(begin_ts, end_ts+1):
        ue_flag[j] = 0
    flag[i] = ue_flag

# get the per-second cumulative sent bytes
def get_throughput(fname):
    ttis = [begin_ts]
    while ttis[-1] + period <= end_ts:
        ttis.append(ttis[-1] + period)

    cumu_bytes = {} # {"ue": {"tti": cumulative bytes}}
    cumu_rbs = {} # {"ue": {"tti": cumulative rbs}}
    per_ue_thr = {} # {"ue": {"tti": per-UE throughput}}

    with open(fname, "r") as fin:
        for line in fin:
            words = line.split(" ")
            if not words[0].isdigit():
                continue
            try:
                if words[1] and words[1] != "app:":
                    continue
            except IndexError:
                continue
            
            tti = int(words[0])
            if tti > end_ts:
                break
            if tti > begin_ts:
                flow = int(words[2])
                if flow not in cumu_bytes:
                    cumu_bytes[flow] = {}
                    cumu_rbs[flow] = {}
                    per_ue_thr[flow] = {}
                cumu_rbs[flow][tti] = int( words[6] )
                cumu_bytes[flow][tti] = int( words[4] )
                flag[flow][tti] = 1

        
            for flow_i in range(n_users):
                if flag[flow_i][tti] == 0:
                    if flow_i not in cumu_bytes:
                        cumu_bytes[flow_i] = {tti: 0}
                        cumu_rbs[flow_i] = {tti: 0}
                        per_ue_thr[flow_i] = {}
                        flag[flow_i][tti] = 1
                    else:
                        cumu_bytes[flow_i][tti] = cumu_bytes[flow_i][tti - 1]
                        cumu_rbs[flow_i][tti] = cumu_rbs[flow_i][tti - 1]
                        flag[flow_i][tti] = 1

    
    for tti in ttis: # no begin_ts
        if tti == begin_ts:
            continue
        print("tti:", tti)
        for flow in cumu_bytes:
            # if tti == 3100:
            #     print("  flow:", flow)
            #     print("  cumu_bytes[flow][tti]:", cumu_bytes[flow])
            per_ue_thr[flow][tti] = (cumu_bytes[flow][tti] - cumu_bytes[flow][tti - period]) * 8 / 1000 / 1000 # Mbps

    return cumu_rbs, cumu_bytes, per_ue_thr, ttis

# thoughput per ue
cumu_rbs, cumu_bytes, per_ue_thr, ttis = get_throughput(folder + "/" + filename)

# for ue in per_ue_thr:
#     print("UE: ", ue, " throughput: ", per_ue_thr[ue])
    
x = [i/1000 for i in ttis[1:]]

for ue in per_ue_thr:
    y = []
    for i in per_ue_thr[ue]:
        y.append(per_ue_thr[ue][i])
    plt.plot(x, y, label="UE"+str(ue))

plt.xlabel("time (s)")
plt.ylabel("throughput (Mbps)")
plt.legend()
plt.show()
plt.savefig("tets_per_ue_throughput.png")

# difference between throughput and GBR
gbr = [2, 0.2, 2, 0.2, 0.2, 0.2, 0.2, 2, 2, 2, 0.2, 2, 2, 2, 2, 2, 20, 2, 0.2, 0.2, 0.2, 2, 0.2, 0.2, 0.2, 2, 2, 2, 2, 20, 2, 2, 0.2, 2, 0.2, 0.2, 20, 2, 2, 2, 0.2, 2, 2, 2, 10, 2, 0.2, 0.2, 0.2, 0.2, 2, 0.2, 0.2, 2, 0.2, 0.2, 0.2, 0.2, 0.2, 2, 2, 0.2, 0.2, 0.2, 2, 2, 0.2, 0.2, 0.2, 20, 2, 2, 2, 2, 2, 0.2, 0.2, 2, 2, 2, 20, 2, 2, 2, 0.2, 10, 10, 0.2, 2, 0.2, 2, 10, 2, 2, 2, 0.2, 0.2, 2, 2, 10]
gbr_result = {}
for i in gbr:
    if i not in gbr_result:
        gbr_result[i] = {"cnt": 0, "acheived": {}, "penalty": 0, "diff": 0}
    gbr_result[i]["cnt"] += 1

for i in range(n_users):
    tti_thr_pair = per_ue_thr[i]
    for tti in tti_thr_pair:
        if tti not in gbr_result[gbr[i]]["acheived"]:
            gbr_result[gbr[i]]["acheived"][tti] = 0
            gbr_result[gbr[i]]["penalty"] = 0
        if tti_thr_pair[tti] >= gbr[i] *0.99:
            gbr_result[gbr[i]]["acheived"][tti] += 1
        else:
            print("i", i, "tti:", tti, "tti_thr_pair[tti]:", tti_thr_pair[tti], "gbr[i]:", gbr[i])
            #print("per_ue_thr[ue]:", per_ue_thr[i])
        gbr_result[gbr[i]]["penalty"] += abs(tti_thr_pair[tti] - gbr[i]) / gbr[i]

    # gbr_result[gbr[i]]["diff"] += abs(per_ue_thr[i][end_ts] - grb[i])
    
    # diff.append(per_ue_thr[i][end_ts] - grb[i])
    # penalty.append(abs(per_ue_thr[i][end_ts] - grb[i]) / grb[i])
    # if per_ue_thr[i][end_ts] >= grb[i]:
    #     acheived.append(1)
    # else:
    #     acheived.append(0)

# print("gbr_result:", gbr_result)
x = [i+1 for i in range(len(gbr_result[gbr[i]]["acheived"].keys()))]
# print(x)
# y = []
# for i in gbr_result[gbr[i]]["acheived"].keys()
#     y.append(gbr_result[gbr[i]]["acheived"][i])

# plot
# Extract x values (timeslots) and y values (achieved values for each GBR)
timeslots = [1100, 2100, 3100, 4100, 5100, 6100, 7100, 8100, 9100, 10100]
gbr_values = [0.2, 2, 10, 20]

# Create a bar chart
bar_width = 0.2
index = np.arange(len(timeslots))

fig, ax = plt.subplots()

for i, gbr in enumerate(gbr_values):
    achieved = [gbr_result[gbr]['acheived'][t] / gbr_result[gbr]["cnt"] * 100 for t in timeslots]
    #penalty = [gbr_result[gbr]['acheived'][t] / gbr_result[gbr]["cnt"] * 100 for t in timeslots]
    ax.bar(index + i * bar_width, achieved, bar_width, label=f'GBR {gbr}')
    print("gbr:", gbr, "achieved:", achieved)

# Add labels and title
ax.set_xlabel('Timeslots (s)')
ax.set_ylabel('penalty (%)')
ax.set_title(scheme + ' - Achieved Throughput per GBR across Timeslots')
ax.set_xticks(index + bar_width * 1.5)
ax.set_xticklabels(x)
ax.legend()

# Show the plot
plt.show()
# save
plt.savefig("tets_per_ue_penalty_rate-" + scheme + ".png")

print("saved")

