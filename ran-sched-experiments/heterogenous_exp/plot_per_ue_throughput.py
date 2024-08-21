# Plot the throughput of UE compared with its GBR
# x: time
# y: throughput

import os
import matplotlib.pyplot as plt
import numpy as np


folder = "configs" #July8-Test2-100UEs
#filenames = ["ours_0.log", "ours_maxcell_0.log", "greedy_edf_0.log", "maxcell_0.log", "maxcell_capped_0.log", "nvs_0.log"]
filenames = ["ours_0.log", "greedy_edf_0.log", "maxcell_0.log", "maxcell_capped_0.log"]
#filenames = ["maxcell_capped_0.log"]

# difference between throughput and GBR
# gbr = [2, 0.2, 2, 0.2, 0.2, 0.2, 0.2, 2, 2, 2, 0.2, 2, 2, 2, 2, 2, 20, 2, 0.2, 0.2, 0.2, 2, 0.2, 0.2, 0.2, 2, 2, 2, 2, 20, 2, 2, 0.2, 2, 0.2, 0.2, 20, 2, 2, 2, 0.2, 2, 2, 2, 10, 2, 0.2, 0.2, 0.2, 0.2, 2, 0.2, 0.2, 2, 0.2, 0.2, 0.2, 0.2, 0.2, 2, 2, 0.2, 0.2, 0.2, 2, 2, 0.2, 0.2, 0.2, 20, 2, 2, 2, 2, 2, 0.2, 0.2, 2, 2, 2, 20, 2, 2, 2, 0.2, 10, 10, 0.2, 2, 0.2, 2, 10, 2, 2, 2, 0.2, 0.2, 2, 2, 10]
# n_users = 100
# gbr_values = [0.2, 2, 10, 20]

# gbr = [2, 20, 20, 20, 20, 10, 30, 2, 20, 30, 10, 20, 2, 20, 30, 10, 10, 20, 2, 30, 2, 30]
# n_users = 22
# gbr_values = [2, 10, 20, 30]

# gbr = [2, 5, 13, 2, 13, 5, 20, 5, 2, 13, 5, 2, 20, 16, 13, 2, 2, 5, 2, 2, 2, 13, 5, 2, 2, 5, 5, 16, 8, 2, 13, 5, 16, 2, 20, 13, 5, 2, 13, 5, 8, 5, 5, 5, 5, 13, 8, 20, 8, 5, 2, 2, 8, 2, 2, 2, 13, 13, 20, 5, 2, 2, 5, 5, 2, 5, 5, 2, 2, 13, 13, 13, 13, 5, 2, 13, 2, 8, 2, 2, 2, 13, 5, 5, 2, 5, 5, 8, 5, 5, 16, 5, 8, 8, 5, 16, 13, 13, 13, 8]
# n_users = 100
# gbr_values = [2, 5, 8, 13, 16, 20]

# gbr: 1->100
# gbr = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100]
# n_users = 100
# gbr_values = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100]

# gbr = [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20]
# n_users = 20
# gbr_values = [10, 20]

gbr = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20]
n_users = 30
gbr_values = [5, 10, 20]

# gbr = [40, 40, 40, 40, 40, 40, 40, 40, 40, 40]
# n_users = 10
# gbr_values = [40]

# gbr = [30, 30, 30, 30, 30, 30, 30, 30, 30, 30]
# n_users = 10
# gbr_values = [30]

# gbr = [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]
# n_users = 40
# gbr_values = [10]


begin_ts = 100
end_ts = 10000 + begin_ts # end time of the measurement
period = 1000 #1000 # throughput measurement window size: number of TTIs
# Extract x values (timeslots) and y values (achieved values for each GBR)
timeslots = [] #[1100, 2100, 3100, 4100, 5100, 6100, 7100, 8100, 9100, 10100]
ts = begin_ts
while ts + period <= end_ts:
    ts += period
    timeslots.append(ts)
print("begin_ts:", begin_ts, "end_ts:", end_ts)
print("timeslots:", timeslots)
# 1s = 1000ms
# 1000 TTIs = 1s


for filename in filenames:
    scheme = filename.split(".")[0][:-2]
    penalty_sum = 0
    
    #print("\nscheme:", scheme)
    # get the per-second cumulative sent bytes
    def get_throughput(fname):
        ttis = [begin_ts]
        while ttis[-1] + period <= end_ts:
            ttis.append(ttis[-1] + period)

        cumu_bytes = {} # {"ue": {"tti": cumulative bytes}}
        cumu_rbs = {} # {"ue": {"tti": cumulative rbs}}
        per_ue_thr = {} # {"ue": {"tti": per-UE throughput}}

        flag  = {}
        for i in range(n_users):
            ue_flag = {}
            for j in range(begin_ts, end_ts+1):
                ue_flag[j] = 0
            flag[i] = ue_flag
            cumu_bytes[i] = {begin_ts-1: 0}
            cumu_rbs[i] = {begin_ts-1: 0}
            per_ue_thr[i] = {}

        total_satisfied_users = 0
        test = 0

        last_tti = begin_ts - 1
        with open(fname, "r") as fin:
            for line in fin:
                if "TTI total satisfied_users" in line:
                    total_satisfied_users += int(line.split(" ")[-1])
                    test += 1

                words = line.split(" ")
                if not words[0].isdigit():
                    continue
                try:
                    if words[1] and words[1] != "app:":
                        continue
                except IndexError:
                    continue
                
                tti = int(words[0])
                #print("tti:", tti)
                if tti < begin_ts:
                    continue
                if tti > end_ts:
                    break
                #print(" then tti:", tti)
                if tti >= begin_ts:
                    flow = int(words[2])
                    cumu_rbs[flow][tti] = int( words[6] )
                    cumu_bytes[flow][tti] = int( words[4] )
                    #print(flow, tti)
                    flag[flow][tti] = 1

                    while last_tti + 1 < tti:
                        for flow_i in range(n_users):
                            if flag[flow_i][last_tti + 1] == 0:
                                cumu_bytes[flow_i][last_tti + 1] = cumu_bytes[flow_i][last_tti]
                                cumu_rbs[flow_i][last_tti + 1] = cumu_rbs[flow_i][last_tti]
                                flag[flow_i][last_tti + 1] = 1
                        last_tti += 1
            
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

                last_tti = tti

        
        for tti in ttis: # no begin_ts
            if tti == begin_ts:
                continue
            #print("tti:", tti)
            for flow in cumu_bytes:
                per_ue_thr[flow][tti] = (cumu_bytes[flow][tti-1] - cumu_bytes[flow][tti - period - 1]) * 8 / (period / 1000) / 1000 / 1000 # Mbps

        print("test", test, " total:", end_ts-begin_ts)
        return cumu_rbs, cumu_bytes, per_ue_thr, ttis, total_satisfied_users

    # thoughput per ue
    cumu_rbs, cumu_bytes, per_ue_thr, ttis, total_satisfied_users = get_throughput(folder + "/" + filename)

    # for ue in per_ue_thr:
    #     print("UE: ", ue, " throughput: ", per_ue_thr[ue])
        
    x = [i/period for i in ttis[1:]]

    sum_thr = 0
    for ue in per_ue_thr:
        y = []
        for i in per_ue_thr[ue]:
            y.append(per_ue_thr[ue][i])
        #plt.plot(x, y, label="UE"+str(ue))
        sum_thr += sum(y)

    # plt.xlabel("time (s)")
    # plt.ylabel("throughput (Mbps)")
    # plt.title(scheme + ' - per_ue_throughput')
    # plt.legend()
    # plt.show()
    # plt.savefig(scheme + " - per_ue_throughput.png")

    gbr_result = {}
    for i in gbr:
        if i not in gbr_result:
            gbr_result[i] = {"cnt": 0, "acheived": {}, "penalty": 0, "diff": 0}
        gbr_result[i]["cnt"] += 1

    for i in range(n_users):
        tti_thr_pair = per_ue_thr[i]
        penalty = 0
        # print("user", i)
        for tti in tti_thr_pair:
            if tti not in gbr_result[gbr[i]]["acheived"]:
                gbr_result[gbr[i]]["acheived"][tti] = 0
                gbr_result[gbr[i]]["penalty"] = 0
            if tti_thr_pair[tti] >= gbr[i]:
                gbr_result[gbr[i]]["acheived"][tti] += 1
            # else:
            #     print("i", i, "tti:", tti, "tti_thr_pair[tti]:", tti_thr_pair[tti], "gbr[i]:", gbr[i])
                #print("per_ue_thr[ue]:", per_ue_thr[i])
            # print("tti:", tti, "tti_thr_pair[tti]:", tti_thr_pair[tti])
            penalty += abs(tti_thr_pair[tti] - gbr[i]) / gbr[i]
            # print("penalty:", abs(tti_thr_pair[tti] - gbr[i]) / gbr[i])
        #print penalty
        # for i in len(gbr):
        #     print("gbr[i]:", gbr[i], "penalty:", gbr_result[i]["penalty"]) 
        #print("penalty sum:", penalty/len(tti_thr_pair))
        penalty_sum += penalty/len(tti_thr_pair)

    # print("penalty_sum:", penalty_sum/n_users)

    x = [i+1 for i in range(len(gbr_result[gbr[i]]["acheived"].keys()))]

    # Create a bar chart
    bar_width = 0.2
    index = np.arange(len(timeslots))

    fig, ax = plt.subplots()

    total_achieved = 0
    for i, g in enumerate(gbr_values):
        #print("filename:", filename)
        # print("gbr:", g, "acheived:", gbr_result[g]['acheived'])
        achieved = [gbr_result[g]['acheived'][t] / gbr_result[g]["cnt"] * 100 for t in timeslots]
        #penalty = [gbr_result[gbr]['acheived'][t] / gbr_result[gbr]["cnt"] * 100 for t in timeslots]
        ax.bar(index + i * bar_width, achieved, bar_width, label=f'GBR {g}')
        total_achieved += sum([gbr_result[g]['acheived'][t] for t in timeslots])
        #print("gbr:", g, "achieved:", achieved)

    print("\n", scheme, "\ntotal_achieved:", total_achieved, " ", total_achieved / (len(timeslots) * n_users) * 100, "%")

    # Add labels and title
    ax.set_xlabel('Timeslots (s)')
    ax.set_ylabel('Satisfication Rate (%)')
    ax.set_title(scheme + ' - Overall Satisfication Rate:' + str(total_achieved / (len(timeslots) * n_users) * 100) + '%')
    ax.set_xticks(index + bar_width * 1.5)
    ax.set_xticklabels(x)
    ax.legend()

    # Show the plot
    plt.show()
    # save
    plt.savefig("test_per_ue_satisfication_rate-" + scheme + ".png")

    print("sum_thr:", sum_thr / ((end_ts - begin_ts)/period), "Mbps")
    print("total_satisfied_users:", total_satisfied_users, " total:", n_users*(end_ts-begin_ts), " rate:", total_satisfied_users / (n_users*(end_ts-begin_ts)) * 100, "%")


