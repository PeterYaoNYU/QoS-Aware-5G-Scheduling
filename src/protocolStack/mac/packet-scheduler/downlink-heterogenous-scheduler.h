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
  
  // Jiajin: initialize pre_defined_gbr as an array of 0 with size of 11. 11 is the number of UEs in the experiment
  // Initialize as (5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55) * 125 = (625, 1250, 1875, 2500, 3125, 3750, 4375, 5000, 5625, 6250, 6875)
  std::vector<double> pre_defined_gbr_ = {625, 1250, 1875, 2500, 3125, 3750, 4375, 5000, 5625, 6250}; // 10 UEs
  //std::vector<double> pre_defined_gbr_ = {1250, 2500, 3750, 5000, 6250, 7500, 8750, 10000}; //, 11250, 12500}; // 10 UEs


  const double beta_ = 0.1;

  // Charlie: the inter-slice scheduling metric (objective)
  // peter: a function pointer that points to the inter slice algorithm
  double (*inter_metric_)(UserToSchedule*, int);

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
  std::vector<int> GetSortedUEsIDbyQoS(std::map<int, double> user_qos_map); // byDDL or byGBR: from min to max
  //vector<int> RBsAllocation_EDF(int num_rbs, UsersToSchedule* user, vector<int> rb_allocation);

};

#endif /* DOWNLINKHETEROGENOUSSCHEDULER_H_ */
