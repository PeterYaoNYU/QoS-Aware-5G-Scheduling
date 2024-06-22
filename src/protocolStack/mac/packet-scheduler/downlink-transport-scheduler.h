/* project: RadioSaber; Mode: C++
 * Copyright (c) 2021, 2022, 2023, 2024 University of Illinois Urbana Champaign
 *
 * This file is part of RadioSaber, which is a project built upon LTE-Sim in 2022
 *
 * RadioSaber is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RadioSaber is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for ore details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Yongzhou Chen <yongzhouc@outlook.com>
 */

#ifndef DOWNLINKTRANSPORTSCHEDULER_H_
#define DOWNLINKTRANSPORTSCHEDULER_H_

#include <vector>
#include <map>
#include "packet-scheduler.h"
#include <deque>

class DownlinkTransportScheduler : public PacketScheduler {
 private:
  // below use customizable scheduler params
  int num_slices_ = 1;
  std::vector<int> user_to_slice_;
  std::vector<double> slice_weights_;
  std::vector<SchedulerAlgoParam> slice_algo_params_;
  std::vector<int> slice_priority_;
  std::vector<double> slice_rbs_offset_;

  // Peter: Keep a running score to measure how well each request is satisfied. 
  std::vector<double> slice_score_;

  // Peter: Store the result of the score to a file
  void logScore();

  const double beta_ = 0.1;
  int inter_sched_ = 0;

  // Charlie: the inter-slice scheduling metric (objective)
  // peter: a function pointer that points to the inter slice algorithm
  double (*inter_metric_)(UserToSchedule*, int, int);

  // peter: count the weight of different slices
  std::vector<double> inter_algo_weight_count_;

  // peter: a flag indicating whether the inter-slice scheduling is in mix mode
  int mix_mode = 0;
  // peter: if cap is set to 1, stop allocating when enough has been allocated through maxcell
  int cap = 0;

 public:
  DownlinkTransportScheduler(std::string config_fname, int algo, int metric);
  virtual ~DownlinkTransportScheduler();

  void SelectFlowsToSchedule();

  virtual void DoSchedule(void);
  virtual void DoStopSchedule(void);

  virtual void RBsAllocation();
  virtual double ComputeSchedulingMetric(UserToSchedule* user,
                                         double spectralEfficiency);
  void UpdateAverageTransmissionRate(void);

  // peter: static method to get TB size from sinr values
  int EstimateTBSizeByEffSinr(std::vector<double> estimatedSinrValues, int rbg_size);
  // peter: for preliminary code, use this vector for GBR:
  // std::vector<double> pre_defined_gbr_ = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55}; // Mbps
  // read the predefined gbr instead from the json file
  std::vector<double> pre_defined_gbr_;
  std::vector<int> pre_defined_gbr_bits_per_second = {};

  // with the new version, we adopt an experiment where the sliding window is disjoint 
  std::vector<int> dataToTransmitInWindow; //Jiajin 0617

  // if disjoint sliding window
  const int WINDOW_SIZE = 1000;
  int remaining_window = 1;//Jiajin 0617

  std::vector<int> GetSortedUEsIDbyQoS(std::map<int, double> user_qos_map, std::vector<std::deque<double>>& allocation_logs, double threshold, int total_rbgs_to_allocate); // byDDL or byGBR: from min to max
  //vector<int> RBsAllocation_EDF(int num_rbs, UsersToSchedule* user, vector<int> rb_allocation);
  int EstimateTBSizeByEffSinr(std::vector<double> estimatedSinrValues, int num_rb, int rbg_size);
};

#endif /* DOWNLINKPACKETSCHEDULER_H_ */
