/* 
 * By Jiajin
 * project: RadioSaber; Mode: C++
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
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Yongzhou Chen <yongzhouc@outlook.com>
 */

#ifndef DOWNLINKHETEROGENOUSSCHEDULER_H_
#define DOWNLINKHETEROGENOUSSCHEDULER_H_

#include <vector>
#include <map>
#include <deque>
#include "packet-scheduler.h"

class DownlinkHeterogenousScheduler : public PacketScheduler {
 private:
  // below use customizable scheduler params
  int num_slices_ = 1;
  std::vector<int> user_to_slice_;
  std::vector<double> slice_weights_;
  std::vector<SchedulerAlgoParam> slice_algo_params_;
  std::vector<int> slice_priority_;
  std::vector<double> slice_rbs_offset_;

  const double beta_ = 0.1;

  // Charlie: the inter-slice scheduling metric (objective)
  // peter: a function pointer that points to the inter slice algorithm
  double (*inter_metric_)(UserToSchedule*, int);

  // Peter: Sliding window to keep track of how many RBs have been allocated to each UE already
  const int WINDOW_SIZE = 1000;
  int num_windows_; 
  std::vector<std::deque<double>> allocation_logs_;

 public:
  DownlinkHeterogenousScheduler(std::string config_fname);
  virtual ~DownlinkHeterogenousScheduler();

  void SelectFlowsToSchedule();

  virtual void DoSchedule(void);
  virtual void DoStopSchedule(void);

  virtual void RBsAllocation();
  virtual double ComputeSchedulingMetric(UserToSchedule* user,
                                         double spectralEfficiency);
  void UpdateAverageTransmissionRate(void);

  // Jiajin add
  std::vector<int> GetSortedUEsIDbyQoS(std::map<int, double> user_qos_map, std::vector<std::deque<double>>& allocation_logs, double threshold); // byDDL or byGBR: from min to max
  //vector<int> RBsAllocation_EDF(int num_rbs, UsersToSchedule* user, vector<int> rb_allocation);

};

#endif /* DOWNLINKHETEROGENOUSSCHEDULER_H_ */
