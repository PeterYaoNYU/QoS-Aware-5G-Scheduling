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
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Yongzhou Chen <yongzhouc@outlook.com>
 */

#include "downlink-heterogenous-scheduler.h"
#include <jsoncpp/json/json.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <numeric>
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../device/ENodeB.h"
#include "../../../device/NetworkNode.h"
#include "../../../flows/MacQueue.h"
#include "../../../flows/application/Application.h"
#include "../../../flows/radio-bearer.h"
#include "../../../load-parameters.h"
#include "../../../phy/lte-phy.h"
#include "../../../protocolStack/mac/AMCModule.h"
#include "../../../protocolStack/rrc/rrc-entity.h"
#include "../../../utility/eesm-effective-sinr.h"
#include "../../packet/Packet.h"
#include "../../packet/packet-burst.h"
#include "../mac-entity.h"
#include "../../../flows/QoS/QoSParameters.h"


using std::unordered_map;
using std::vector;
using std::map;

// Peter: The coordinates are likely to represent the RBG (i) and the slice index (j).
using coord_t = std::pair<int, int>;
using coord_cqi_t = std::pair<coord_t, double>;

DownlinkHeterogenousScheduler::DownlinkHeterogenousScheduler(
    std::string config_fname) {
  std::cerr << "DownlinkHeterogenousScheduler::DownlinkHeterogenousScheduler" << std::endl;
  std::ifstream ifs(config_fname);
  if (!ifs.is_open()) {
    throw std::runtime_error("Fail to open configuration file.");
  }
  Json::Reader reader;
  Json::Value obj;
  reader.parse(ifs, obj);
  ifs.close();
  const Json::Value& ues_per_slice = obj["ues_per_slice"];
  num_slices_ = ues_per_slice.size();
  int num_ue;
  for (int i = 0; i < num_slices_; i++) {
    // peter: get the number of users per slice
    num_ue = ues_per_slice[i].asInt();
    for (int j = 0; j < num_ue; j++) {
      // peter: map user to slice vector, important to differentiate between the i and j here.
      user_to_slice_.push_back(i);
    }
  }
  const Json::Value& slice_schemes = obj["slices"];
  for (int i = 0; i < slice_schemes.size(); i++) {
    int n_slices = slice_schemes[i]["n_slices"].asInt();
    for (int j = 0; j < n_slices; j++) {
      slice_algo_params_.emplace_back(slice_schemes[i]["algo_alpha"].asInt(),
                                slice_schemes[i]["algo_beta"].asInt(),
                                slice_schemes[i]["algo_epsilon"].asInt(),
                                slice_schemes[i]["algo_psi"].asInt(),
                                slice_schemes[i]["type"].asInt());
      // Peter: this is a useful place to get the enterprise scheduler algorithms parameters.
      slice_weights_.push_back(slice_schemes[i]["weight"].asDouble());
      // Peter Remark: I think this should be the right place to place 
      // bit rate / delay / type of the UE etc. If such parameters come along with the Slice instead of the UE
    }
  }
  // [peter] for each slice, calculate the priority
  slice_priority_.resize(num_slices_);
  std::fill(slice_priority_.begin(), slice_priority_.end(), 0);
  // [Peter] used later to ensure that the allocations of rbgs overtime is overall fair and accords to the SLA
  slice_rbs_offset_.resize(num_slices_); 
  std::fill(slice_rbs_offset_.begin(), slice_rbs_offset_.end(), 0);


  // [Peter] Initialize the sliding window number, and reserve the corresponding deque space
  num_windows_ = user_to_slice_.size();
  fprintf(stderr, "num_windows_: %d\n", num_windows_);
  allocation_logs_.resize(num_windows_);

  SetMacEntity(0);
  CreateUsersToSchedule();
}

DownlinkHeterogenousScheduler::~DownlinkHeterogenousScheduler() {
  Destroy();
}

// [Peter]update the CQI, call the actual scheduling function
// [Peter]entrance of scheduling
void DownlinkHeterogenousScheduler::DoSchedule(void) {
#ifdef SCHEDULER_DEBUG
  std::cout << "\nStart DL packet scheduler for node "
            << GetMacEntity()->GetDevice()->GetIDNetworkNode() << std::endl;
#endif
  // std::cerr << GetTimeStamp() << " Start DL packet scheduler for node "
  //           << GetMacEntity()->GetDevice()->GetIDNetworkNode() << std::endl;

  // [Peter] update each bearer's average transmission rate
  UpdateAverageTransmissionRate(); //Jiajin: May do not need it anymore
  SelectFlowsToSchedule();


  // std::cerr << GetTimeStamp() << " GetUsersToSchedule()->size(): "
            // << GetUsersToSchedule()->size() << std::endl;
  if (GetUsersToSchedule()->size() != 0) {
      RBsAllocation();
  }

  DoStopSchedule();
}

// peter: actually send out the packets.
void DownlinkHeterogenousScheduler::DoStopSchedule(void) {
  PacketBurst* pb = new PacketBurst();
  UsersToSchedule* uesToSchedule = GetUsersToSchedule();
  for (auto it = uesToSchedule->begin(); it != uesToSchedule->end(); it++) {
    UserToSchedule* user = *it;
    int availableBytes = user->GetAllocatedBits() / 8;
    // let's not reallocate RBs between flows firstly
    for (int i = MAX_BEARERS - 1; i >= 0; i--) {
      if (availableBytes <= 0)
        break;
      if (user->m_dataToTransmit[i] > 0) {
        assert(user->m_bearers[i] != NULL);
        int dataTransmitted = min(availableBytes, user->m_dataToTransmit[i]);
        availableBytes -= dataTransmitted;
        user->m_bearers[i]->UpdateTransmittedBytes(dataTransmitted);
        user->m_bearers[i]->UpdateCumulateRBs(
            user->GetListOfAllocatedRBs()->size());
        std::cerr << GetTimeStamp() << " app: "
                  << user->m_bearers[i]->GetApplication()->GetApplicationID()
                  << " cumu_bytes: " << user->m_bearers[i]->GetCumulateBytes()
                  << " cumu_rbs: " << user->m_bearers[i]->GetCumulateRBs()
                  << " hol_delay: "
                  << user->m_bearers[i]->GetHeadOfLinePacketDelay()
                  << " user: " << user->GetUserID()
                  << " slice: " << user_to_slice_[user->GetUserID()]
                  << std::endl;

        RlcEntity* rlc = user->m_bearers[i]->GetRlcEntity();
        PacketBurst* pb2 = rlc->TransmissionProcedure(dataTransmitted);

        if (pb2->GetNPackets() > 0) {
          std::list<Packet*> packets = pb2->GetPackets();
          std::list<Packet*>::iterator it;
          for (it = packets.begin(); it != packets.end(); it++) {
            Packet* p = (*it);
            pb->AddPacket(p->Copy());
          }
        }
        delete pb2;
      }
    }
  }
  UpdateTimeStamp();

  GetMacEntity()->GetDevice()->SendPacketBurst(pb);
}


void DownlinkHeterogenousScheduler::UpdateAverageTransmissionRate(void) {
  // we should update the user average transmission rate instead of the flow transmission rate
  RrcEntity* rrc =
      GetMacEntity()->GetDevice()->GetProtocolStack()->GetRrcEntity();
  RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer();

  for (std::vector<RadioBearer*>::iterator it = bearers->begin();
       it != bearers->end(); it++) {
    RadioBearer* bearer = (*it);
    bearer->UpdateAverageTransmissionRate();
  }
}

// [Peter] For each radio bearer in each UE, get the respective spectral efficiency
// [Peter] update the slice's priority to that of the highest UE's priority in the slice
// peter: clear all previous users, then readd new users to the scheduler
void DownlinkHeterogenousScheduler::SelectFlowsToSchedule() {
#ifdef SCHEDULER_DEBUG
  std::cout << "\t Select Flows to schedule" << std::endl;
#endif

  ClearUsersToSchedule();
  // Peter: RRC stands for radio resource control, it is the place where the slice context is stored
  RrcEntity* rrc =
      GetMacEntity()->GetDevice()->GetProtocolStack()->GetRrcEntity();
  // Peter: this is unexpected, because according to the paper, radio bearer is managed by individual UE,
  //  but here it seems to suggest that
  // radio bearers are managed by thr RRC
  RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer();

  // [Peter] Fill the slice priority vector with 0s
  std::fill(slice_priority_.begin(), slice_priority_.end(), 0);
  for (std::vector<RadioBearer*>::iterator it = bearers->begin();
       it != bearers->end(); it++) {
    //SELECT FLOWS TO SCHEDULE
    RadioBearer* bearer = (*it);

    // get the amount of data that the UE wants to transmit
    if (bearer->HasPackets() &&
        bearer->GetDestination()->GetNodeState() == NetworkNode::STATE_ACTIVE) {
      //compute data to transmit
      int dataToTransmit;
      if (bearer->GetApplication()->GetApplicationType() ==
          Application::APPLICATION_TYPE_INFINITE_BUFFER) {
        dataToTransmit = 100000000;
      } else {
        dataToTransmit = bearer->GetQueueSize();
      }

      //compute spectral efficiency
      ENodeB* enb = (ENodeB*)GetMacEntity()->GetDevice();
      // [Peter]Get the previous record of each ue
      ENodeB::UserEquipmentRecord* ueRecord = enb->GetUserEquipmentRecord(
          bearer->GetDestination()->GetIDNetworkNode());
      std::vector<double> spectralEfficiency;
      std::vector<int> cqiFeedbacks = ueRecord->GetCQI();
      // Peter: this is important, each UE has multiple CQI, each for a different frequency
      int numberOfCqi = cqiFeedbacks.size();
      // peter: get the Adaptive Modulation And Coding Scheme, which contains the method to convert CQI into spectral efficiency
      AMCModule* amc = GetMacEntity()->GetAmcModule();
      // [Peter]calculate spectral efficiency based on CQI
      for (int i = 0; i < numberOfCqi; i++) {
        spectralEfficiency.push_back(
            amc->GetEfficiencyFromCQI(cqiFeedbacks.at(i)));
      }

      // create flow to scheduler record
      // [Peter]update the slice priority to be the biggest of all UEs in that slice
      int slice_id = user_to_slice_[bearer->GetUserID()];
      if (bearer->GetPriority() > slice_priority_[slice_id]) {
        slice_priority_[slice_id] = bearer->GetPriority();
      }
      InsertFlowToUser(bearer, dataToTransmit, spectralEfficiency,
                       cqiFeedbacks);
    }
  }
}

int DownlinkHeterogenousScheduler::EstimateTBSizeByEffSinr(std::vector<double> estimatedSinrValues, int num_rbg, int rbg_size) { 
  if (num_rbg == 0) {
    return 0;
  }
  AMCModule* amc = GetMacEntity()->GetAmcModule();
  double effectiveSinr = GetEesmEffectiveSinr(estimatedSinrValues);
  int mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr));
  int transportBlockSize = amc->GetTBSizeFromMCS(mcs, num_rbg * rbg_size);
  return transportBlockSize;
}

int DownlinkHeterogenousScheduler::EstimateTBSizeByEffSinr(std::vector<double> estimatedSinrValues, int rbg_size) { 
  int num_rbg = estimatedSinrValues.size();
  if (num_rbg == 0) {
    return 0;
  }
  AMCModule* amc = GetMacEntity()->GetAmcModule();
  double effectiveSinr = GetEesmEffectiveSinr(estimatedSinrValues);
  int mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr));
  int transportBlockSize = amc->GetTBSizeFromMCS(mcs, num_rbg * rbg_size);
  return transportBlockSize;
}

bool sortByVal(const std::pair<int, int> &a, const std::pair<int, int> &b) {
    return a.second < b.second; // sort by increasing order of value
}

bool sortByValDesc(const std::pair<int, int> &a, const std::pair<int, int> &b) {
    return a.second > b.second; // sort by decreasing order of value
}




// peter: if there is leftover RBs:
// if there is unsatisfied UEs, 
// for each resource block, find the UE that have the highest throughput AND is not satisfied, under the promise that its performance will not be negatively impacted
// by the newly added resource block. 
// In the unlikey event that all UEs have been satisifed in this TTI, allocate according to the highest throughput
static vector<std::pair<int, int>> AllocateLeftOverRBs_hetero(std::vector<bool>& satisfied_ues, DownlinkHeterogenousScheduler* downlink_transport_scheduler, std::vector<bool>& rbg_availability, int rbg_size, std::map<int, std::vector<double>>& current_sinr_vals)
{
  // @param alloc_res: the return vector of this function
  // the first is the rbg_id, the second is the UE that this leftover RBG should be assigned to 
  std::vector<std::pair<int, int>> alloc_res;
  // check if there is leftover RBs
  bool all_allocated = std::all_of(rbg_availability.begin(), rbg_availability.end(), [](bool val){ return ! val; });
  if (all_allocated) {
    std::cerr << "There is no leftover resource block" << std::endl;
    return alloc_res;
  }

  std::vector<int> candidate_ues;
  std::vector<int> all_ues;
  // check if all UEs have already been satisfied
  bool allTrue = std::all_of(satisfied_ues.begin(), satisfied_ues.end(), [](bool value) {return value; });
  if (allTrue) {
    std::cerr << "all users alreasy satisfied in this TTI " << std::endl;
    // allocate according to the max throughput
    for (auto i = 0 ; i < satisfied_ues.size(); i++) {
      candidate_ues.push_back(static_cast<int>(i));
    }
  } else {
    // get all the unsatisfied UEs out for choices
    std::cerr << "candidate UEs for further allocation: ";
    for (auto i = 0; i < satisfied_ues.size(); i++ ) {
      if (!satisfied_ues[i]) {
        candidate_ues.push_back(static_cast<int>(i));
        std::cerr << i << ", ";
      }
    }
    std::cerr << " " << std::endl;
  }

  for (auto i = 0 ; i < satisfied_ues.size(); i++) {
    all_ues.push_back(static_cast<int>(i));
  }

  // @param candidate_ues hold the ues that should be allocated (either unsatisifed, or all have already been satisfied)
  // allocate to them based on throughput
  // under the premise that it does not lower the overal Sinr
  auto amc = downlink_transport_scheduler->GetMacEntity()->GetAmcModule();
  auto users = downlink_transport_scheduler->GetUsersToSchedule();

  for (int i = 0; i < rbg_availability.size(); i++) {
    if (rbg_availability[i] == true) {
      std::cerr << "Allocating surplus RBG: " << i << std::endl;
      bool allocated_to_unsatified_ue = false;
      // @ channel_quality: for a specific RBG: ue_id -> channel_quality 
      std::vector<std::pair<int, double>> channel_quality ;
      for (auto j = 0; j < candidate_ues.size(); j++) {
        channel_quality.push_back(std::make_pair(candidate_ues[j], amc->GetSinrFromCQI(users->at(candidate_ues[j])->GetCqiFeedbacks().at(i * rbg_size))));
      }
      std::sort(channel_quality.begin(), channel_quality.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b) {
                  return a.second > b.second;
              });
      for (auto iter = channel_quality.begin(); iter != channel_quality.end(); iter++ ) {
        auto ue_id = iter->first;
        auto current_sinr = iter->second;
        auto allocated_sinr_vec = current_sinr_vals[ue_id];
        auto sum = std::accumulate(allocated_sinr_vec.begin(), allocated_sinr_vec.end(), 0.0);
        auto avg_sinr = sum / allocated_sinr_vec.size();
        auto old_TBSize = downlink_transport_scheduler->EstimateTBSizeByEffSinr(allocated_sinr_vec, rbg_size);
        allocated_sinr_vec.push_back(current_sinr);
        auto tentative_TBSize = downlink_transport_scheduler->EstimateTBSizeByEffSinr(allocated_sinr_vec, rbg_size);
        std::cerr<< "rbg id: " << i << " ue id: " << ue_id << " old_tbsize: " << old_TBSize << " new tbsize: " << tentative_TBSize << " current sinr " << current_sinr << " Avg prev sinr: " << avg_sinr << std::endl;
        if (tentative_TBSize >= old_TBSize) {
          std::cerr << "Allocating the surplus RBs to " << ue_id << " and the RBG id is " << i << std::endl;
          allocated_to_unsatified_ue = true;
          rbg_availability[i] = false;
          alloc_res.push_back(std::make_pair(i, ue_id));
          current_sinr_vals[ue_id] = allocated_sinr_vec;
          int request = int(downlink_transport_scheduler->pre_defined_gbr_[ue_id]) * 1000 * 1000 / 1000; // Mbps -> bits per TTI // TODO: check the User ID
          if (tentative_TBSize > request) {
            candidate_ues.erase(std::remove(candidate_ues.begin(), candidate_ues.end(), ue_id), candidate_ues.end());
            satisfied_ues[ue_id] = true;
          }
          break;
        }
      }

      if (!allocated_to_unsatified_ue) {
        std::cerr << "not allocated to unsatisfieed UE, allocating to the UE with the highest throughput" << std::endl;
        channel_quality.clear();
        for (auto j = 0; j < all_ues.size(); j++) {
          channel_quality.push_back(std::make_pair(j, amc->GetSinrFromCQI(users->at(j)->GetCqiFeedbacks().at(i * rbg_size))));
        }
        std::sort(channel_quality.begin(), channel_quality.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b) {
                    return a.second > b.second;
                });
        for (auto iter = channel_quality.begin(); iter != channel_quality.end(); iter++ ) {
          auto ue_id = iter->first;
          auto current_sinr = iter->second;
          auto allocated_sinr_vec = current_sinr_vals[ue_id];
          auto sum = std::accumulate(allocated_sinr_vec.begin(), allocated_sinr_vec.end(), 0.0);
          auto avg_sinr = sum / allocated_sinr_vec.size();
          auto old_TBSize = downlink_transport_scheduler->EstimateTBSizeByEffSinr(allocated_sinr_vec, rbg_size);
          allocated_sinr_vec.push_back(current_sinr);
          auto tentative_TBSize = downlink_transport_scheduler->EstimateTBSizeByEffSinr(allocated_sinr_vec, rbg_size);
          std::cerr<< "rbg id: " << i << " ue id: " << ue_id << " old_tbsize: " << old_TBSize << " new tbsize: " << tentative_TBSize << " current sinr " << current_sinr << " Avg prev sinr: " << avg_sinr << std::endl;
          if (tentative_TBSize >= old_TBSize) {
            std::cerr << "Allocating the unsatifactory surplus RBs to " << ue_id << " and the RBG id is " << i << std::endl;
            rbg_availability[i] = false;
            alloc_res.push_back(std::make_pair(i, ue_id));
            current_sinr_vals[ue_id] = allocated_sinr_vec;
            break;
          }
        }
      }
    }
  }
  return alloc_res;
}


void DownlinkHeterogenousScheduler::RBsAllocation() {

  // std::cerr << GetTimeStamp() << " ====== RBsAllocation ====== " << std::endl;

  UsersToSchedule* users = GetUsersToSchedule();
  int nb_rbs = GetMacEntity()
                   ->GetDevice()
                   ->GetPhy()
                   ->GetBandwidthManager()
                   ->GetDlSubChannels()
                   .size();
  int rbg_size = get_rbg_size(nb_rbs);
  // currently nb_rbgs should be divisible
  nb_rbs = nb_rbs - (nb_rbs % rbg_size);
  assert(nb_rbs % rbg_size == 0);

  std::map<int, std::vector<double>> current_sinr_vals;

  // find out slices without data/flows at all, and assign correct rb target
  std::vector<bool> slice_with_data(num_slices_, false);
  std::vector<int> slice_target_rbs(num_slices_, 0); // TODO: assume only 3 slices now: 3 types

  int num_nonempty_slices = 0;
  int extra_rbs = nb_rbs;
  // vector<int> users_id_edf; // earliest ddl first
  // vector<int> users_id_gbr; // guranteed bit rate
  // vector<int> users_id_be; // best effort
  for (auto it = users->begin(); it != users->end(); ++it) {
    int user_id = (*it)->GetUserID();
    int slice_id = user_to_slice_[user_id];
    if (slice_with_data[slice_id])
      continue;
    num_nonempty_slices += 1;
    slice_with_data[slice_id] = true;
    slice_target_rbs[slice_id] = (int)(nb_rbs * slice_weights_[slice_id] + slice_rbs_offset_[slice_id]);
    extra_rbs -= slice_target_rbs[slice_id];
    // // Jiajin add: classify UEs
    // switch (slice_algo_params_[slice_id].type)
    // {
    //   case 1: // Delay sensitive
    //     users_id_edf.push_back(user_id);
    //   case 2: // Guaranteed Bit Rate
    //     users_id_gbr.push_back(user_id);
    //   case 3: // Best-Effort
    //     users_id_be.push_back(user_id);
    //   default:
    //     throw std::runtime_error("Error: invalid slice type");
    //     break;
    // }
  }
  // std::cout << "slice target RBs:";
  // for (int i = 0; i < num_slices_; ++i) {
  //   std::cout << "(" << i << ", "
  //     << slice_target_rbs[i] << ", "
  //     << slice_rbs_offset_[i] << ","
  //     << slice_weights_[i] << ") ";
  // }
  // std::cout << std::endl;

  // std::cerr << GetTimeStamp() << " slice target RBs:";
  // for (int i = 0; i < num_slices_; ++i) {
  //   std::cerr << "(" << i << ", "
  //             << slice_target_rbs[i] << ", "
  //             << slice_rbs_offset_[i] << ","
  //             << slice_weights_[i] << ") ";
  // }

  assert(num_nonempty_slices != 0);
  // we enable reallocation between slices, but not flows
  bool is_first_slice = true;
  int rand_begin_idx = rand();
  for (int i = 0; i < num_slices_; ++i) {
    int k = (i + rand_begin_idx) % num_slices_;
    if (slice_with_data[k]) {
      slice_target_rbs[k] += extra_rbs / num_nonempty_slices;
      if (is_first_slice) {
        slice_target_rbs[k] += extra_rbs % num_nonempty_slices;
        is_first_slice = false;
      }
    }
  }
  int nb_rbgs = nb_rbs / rbg_size;
  // calculate the rbg quota for slices
  std::vector<int> slice_quota_rbgs(num_slices_, 0);
  std::vector<int> slice_final_rbgs(num_slices_, 0);
  int extra_rbgs = nb_rbgs;
  for (int i = 0; i < num_slices_; ++i) {
    slice_quota_rbgs[i] = (int)(slice_target_rbs[i] / rbg_size);
    extra_rbgs -= slice_quota_rbgs[i];
  }
  // peter: Distribute any remaining RBs (extra_rbs) evenly across the slices with data.
  // peter: If are leftover RBs after the even distribution, allocate them to the first slice
  // (after randomizing the start index with rand_begin_idx to distribute leftovers randomly).
  is_first_slice = true;
  rand_begin_idx = rand();
  for (int i = 0; i < num_slices_; ++i) {
    int k = (rand_begin_idx + i) % num_slices_;
    if (slice_with_data[k]) {
      slice_quota_rbgs[k] += extra_rbgs / num_nonempty_slices;
      if (is_first_slice) {
        slice_quota_rbgs[k] += extra_rbgs % num_nonempty_slices;
        is_first_slice = false;
      }
    }
  }

  std::cerr << "slice_id, target_rbs, quota_rbgs: ";
  for (int i = 0; i < num_slices_; ++i) {
    std::cerr << "(" << i << ", " << slice_target_rbs[i] << ", "
              << slice_quota_rbgs[i] << ") ";
  }
  std::cerr << std::endl;

  AMCModule* amc = GetMacEntity()->GetAmcModule();
  
  std::cerr << "==== start initialization====" << std::endl;
  std::vector<pair<int, int>> user_requestRB_pair; // user_id -> #rb_needed
  std::vector<pair<int, int>> rbgid_impact_pair; // rbg_id -> impact: number of UEs that can be suitable for
  std::map<int, vector<int>> rbg_impact_ues; // just for record
  bool metrics[nb_rbgs][users->size()]; // 1: the RB suitable for the UE, 0: not suitable
  for (int i = 0; i < nb_rbgs; i++) { // initialization
    rbgid_impact_pair.push_back(make_pair(i, 0));
    for (int j = 0; j < users->size(); j++) {
      metrics[i][j] = 0;
    }
    rbg_impact_ues.insert(make_pair(i, vector<int>()));
  }
  std::cerr << "==== initialized==== metrics[" << nb_rbgs << "][" << users->size() << "] ,rbgid_impact_pair[" << nb_rbgs << "]" << std::endl;
  std::cerr << "==== Estimate UE: # RB needed, RB lower bound, RB impact ====" << std::endl;
  // Sort CQI for each UE & Calculate RB impact 
  for (auto it = users->begin(); it != users->end(); it++) { // per UE, sort its CQIs
    UserToSchedule* user = *it;
    // print GetSortedRBGIds
    std::cerr << " Sorted RB: user_id: " << user->GetUserID() << ", sortedRBIds(" << user->GetSortedRBGIds().size() << "): ";
    for (int i = 0; i < user->GetSortedRBGIds().size(); i++) {
      std::cerr << user->GetSortedRBGIds().at(i) << "(" << user->GetCqiFeedbacks().at(user->GetSortedRBGIds().at(i) * rbg_size) << ") ";
    }
    int num_RBG = 0;
    int available_TBSize = 0;
    vector<double> estimatedSinrValues = {};
    int request = int(pre_defined_gbr_[user->GetUserID()]) * 1000 * 1000 / 1000; // Mbps -> bits per TTI // TODO: check the User ID
    std::cerr << "== user_id: " << user->GetUserID() << ", request:" << request << " ==" << std::endl;
    // find the min number of RB required
    while (available_TBSize < request) { 
      std::cerr << "  available_TBSize= " << available_TBSize << " < request=" << request << std::endl;
      num_RBG += 1; 
      if (num_RBG > nb_rbgs) {
        break;
      }
      int rbg_id = user->GetSortedRBGIds().at(num_RBG-1);
      double sinr = amc->GetSinrFromCQI(user->GetCqiFeedbacks().at(rbg_id * rbg_size)); 
      std::cerr << "  num_RBG=" << num_RBG << ", rbg_id=" << rbg_id << ", sinr=" << sinr << std::endl;
      estimatedSinrValues.push_back(sinr);
      std::cerr << "  estimatedSinrValues: ";
      for (int i = 0; i < estimatedSinrValues.size(); i++) {
        std::cerr << estimatedSinrValues.at(i) << " ";
      }
      available_TBSize = EstimateTBSizeByEffSinr(estimatedSinrValues, num_RBG, rbg_size);
      std::cerr << "  EstimateTBSizeByEffSinr() available_TBSize= " << available_TBSize << ", num_RBG=" << num_RBG << std::endl;
    }
    //user.SetRequiredRBs(num_RBG); // min number of RB required; num_RB=nb_rbgs+1 is possible, which means cannot be satisfied 
    if (request > 0 && num_RBG <= nb_rbgs){
      user_requestRB_pair.push_back(std::make_pair(user->GetUserID(), num_RBG));
      std::cerr << "  final: user->GetUserID()=" << user->GetUserID() << ", num_RBG=" << num_RBG << std::endl;
      // find the lower bound of RB idx for available CQI
      int lower_bound_idx = num_RBG; // lower bound index in sorted_RB
      while (available_TBSize > request && lower_bound_idx < nb_rbgs) {
        std::cerr << "  lower_bound_idx = " << lower_bound_idx << " available_TBSize= " << available_TBSize << " > request=" << request << std::endl;
        lower_bound_idx += 1;
        estimatedSinrValues.pop_back();
        // print estimatedSinrValues
        std::cerr << "  after pop_back, estimatedSinrValues: ";
        for (int i = 0; i < estimatedSinrValues.size(); i++) {
          std::cerr << estimatedSinrValues.at(i) << " ";
        }
        int rbg_id = user->GetSortedRBGIds().at(lower_bound_idx-1);
        double sinr = amc->GetSinrFromCQI(user->GetCqiFeedbacks().at(rbg_id * rbg_size)); 
        std::cerr << " add rbg_id:" << rbg_id << ", lower_bound_idx:" << lower_bound_idx << ", sinr:" << sinr << std::endl;
        estimatedSinrValues.push_back(sinr);
        // print estimatedSinrValues
        std::cerr << "  estimatedSinrValues: ";
        for (int i = 0; i < estimatedSinrValues.size(); i++) {
          std::cerr << estimatedSinrValues.at(i) << " ";
        }
        available_TBSize = EstimateTBSizeByEffSinr(estimatedSinrValues, num_RBG, rbg_size);
        std::cerr << "  EstimateTBSizeByEffSinr() available_TBSize= " << available_TBSize << ", lower_bound_idx=" << lower_bound_idx << std::endl;
      }
      lower_bound_idx -= 1; // lower bound index in sorted_RB
      user->SetLowerBoundSortedIdx(lower_bound_idx-1);
      for (int i = 0; i < lower_bound_idx; i++) {
        int rbg_id = user->GetSortedRBGIds().at(i);
        rbgid_impact_pair[rbg_id].second += 1;
        rbg_impact_ues[rbg_id].push_back(user->GetUserID());
        std::cerr << " rbgid_impact_pair: rbg_id=" << rbg_id << "(" << rbgid_impact_pair[rbg_id].first << "), impact=" << rbgid_impact_pair[rbg_id].second << " rbg_impact_ues.size()=" << rbg_impact_ues[rbg_id].size() << ": ";
        for (int j = 0; j < rbg_impact_ues[rbg_id].size(); j++) {
          std::cerr << rbg_impact_ues[rbg_id][j] << " ";
        }
        std::cerr << std::endl;
        metrics[rbg_id][user->GetUserID()] = 1;
        std::cerr << " metrics: rbg_id=" << rbg_id << ", user_id=" << user->GetUserID() << ", metric=" << metrics[rbg_id][user->GetUserID()] << std::endl;
      }
    }
  }
  
  bool rbg_availability[nb_rbgs]; // 1: available, 0: unavailable
  for (int i = 0; i < nb_rbgs; i++) {
    rbg_availability[i] = 1;
  }
  // ========== Sort UEs by Min #RB required ==========
  sort(user_requestRB_pair.begin(), user_requestRB_pair.end(), sortByVal); // min #rb first
  sort(rbgid_impact_pair.begin(), rbgid_impact_pair.end(), sortByVal); // min impact first

  //print user_requestRB_pair
  std::cerr << "==== user_requestRB_pair ====" << std::endl;
  for (int i = 0; i < user_requestRB_pair.size(); i++) {
    std::cerr << "user_id: " << user_requestRB_pair[i].first << ", num_RBG_needed: " << user_requestRB_pair[i].second << std::endl;
  }
  std::cerr << "==== rbgid_impact_pair ====" << std::endl;
  for (int i = 0; i < nb_rbgs; i++) {
    int rb_idx = rbgid_impact_pair[i].first;
    std::cerr << i << "th rbg_id: " << rbgid_impact_pair[i].first << ", impact: " << rbgid_impact_pair[i].second;
    // print rbg_impact_ues
    std::cerr << "  rbg_impact_ues(" << rbg_impact_ues[rb_idx].size() << "): ";
    for (int j = 0; j < rbg_impact_ues[rb_idx].size(); j++) {
      std::cerr << rbg_impact_ues[rb_idx][j] << " ";
    }
    std::cerr << std::endl;
  }

  std::vector<pair<int, int>> satisfied_users;
  int ue_satisfied[nb_rbgs]; // 1: satisfied, 0: not satisfied
  for (int i = 0; i < nb_rbgs; i++) {
    ue_satisfied[i] = 0;
  }
  for (int i = 0; i < user_requestRB_pair.size(); i++) {
    int user_id = user_requestRB_pair[i].first;
    int num_RBG_needed = user_requestRB_pair[i].second;
    if (num_RBG_needed > nb_rbgs) {
      std::cerr << "Warning, user_id: " << user_id << " cannot be satisfied, required RBGs: " << num_RBG_needed << std::endl;
      continue;
    }
    // find the available RBs
    int allocated_RBG_num = 0;
    // iterate sorted rbgid_impact_pair, find the suitable RBs for the UE
    for (int j = 0; j < nb_rbgs; j++) {
      int rbg_id = rbgid_impact_pair[j].first;
      if (rbg_availability[rbg_id] == 1 && metrics[rbg_id][user_id] == 1) {
        std::cerr << "  Allcoation for user_id: " << user_id << ", rbg_id: " << rbg_id << " rb:[";
        // peter: for allocating surplus RBs
        current_sinr_vals[user_id].push_back(amc->GetSinrFromCQI(users->at(user_id)->GetCqiFeedbacks().at(rbg_id * rbg_size)));
        // rbg_availability[rbg_id] = false;
        int l = rbg_id * rbg_size, r = (rbg_id + 1) * rbg_size;
        for (int j = l; j < r; ++j) {
          users->at(user_id)->GetListOfAllocatedRBs()->push_back(j);
          std::cerr << j << " ";
        }
        users->at(user_id)->GetListOfAllocatedRBGs()->push_back(rbg_id);
        std::cerr << "]" << std::endl;
        rbg_availability[rbg_id] = 0;
        allocated_RBG_num += 1;
        if (allocated_RBG_num == num_RBG_needed) 
        {
          satisfied_users.push_back(make_pair(user_id, allocated_RBG_num));
          ue_satisfied[user_id] = 1;
          break;
        }
      }
    }
  }
  // print those unallocated RB
  std::cerr << "==== unallocated RBs ====" << std::endl;
  for (int i = 0; i < nb_rbgs; i++) {
    int rbg_id = rbgid_impact_pair[i].first;
    if (rbg_availability[rbg_id] == 1) {
      std::cerr << i << "th rbg_id: " << rbgid_impact_pair[i].first << " impact:" << rbgid_impact_pair[i].second;
      // print rbg_impact_ues
      std::cerr << "  rbg_impact_ues(" << rbg_impact_ues[rbg_id].size() << "): ";
      for (int j = 0; j < rbg_impact_ues[rbg_id].size(); j++) {
        std::cerr << rbg_impact_ues[rbg_id][j] << " ";
      }
      std::cerr << std::endl;
    }
  }


  std::cerr << "+++++++++++++==== satisfied_users ====+++++++++++++====" << std::endl;
  for (int i = 0; i < satisfied_users.size(); i++) {
    std::cerr << "user_id: " << satisfied_users[i].first << ", allocated_RBG_num:" << satisfied_users[i].second << std::endl;
  }

  std::vector<bool> satisfied_ues;
  // peter: this looks like a mistake to me. For now, keep it. Debug it later. When the number of UEs is small, does not matter much
  for (auto i = 0; i < users->size(); i++) {
    if (ue_satisfied[i] == 1) {
      satisfied_ues.push_back(true);
    } else {
      satisfied_ues.push_back(false);
    }
  }

  std::vector<bool> rbg_availability_vec;
  for (auto i = 0; i < nb_rbgs; i++) {
    if (rbg_availability[i] == 1) {
      rbg_availability_vec.push_back(true);
    } else {
      rbg_availability_vec.push_back(false);
    }
  }


  auto alloc_res = AllocateLeftOverRBs_hetero(satisfied_ues, this, rbg_availability_vec, rbg_size, current_sinr_vals);

  for (auto iter  = alloc_res.begin(); iter != alloc_res.end(); iter++) {
    int rbg_id = iter->first;
    auto uindex = iter->second;
    int l = rbg_id * rbg_size, r = (rbg_id+1) * rbg_size;
    for (int j = l; j < r; ++j) {
      users->at(uindex)->GetListOfAllocatedRBs()->push_back(j);
    }
  }


  // ========= Allocation for those unallocated RBs: greedy, per UE =========
  for (int uid = 0; uid < users->size(); uid++) {
    if (satisfied_ues[uid] == 1) {
      continue;
    }
    int num_RBG_needed = user_requestRB_pair[uid].second;
    if (num_RBG_needed > nb_rbgs) {
      std::cerr << "Warning, user_id: " << uid << " cannot be satisfied, required RBGs: " << num_RBG_needed << std::endl;
      continue;
    }
    for (int idx = users->at(uid)->GetLowerBoundSortedIdx()+1; idx < nb_rbgs; idx++) {
      int crt_rbg_id = users->at(uid)->GetSortedRBGIds().at(idx);
      if (rbg_availability[crt_rbg_id] == 1) {
        std::cerr << "  Try to allcoation for user_id: " << uid << ", rbg_id: " << crt_rbg_id;
        std::vector<double> new_estimatedSinrValues = {};
        for (int alloc_rb_id = 0; alloc_rb_id < users->at(uid)->GetListOfAllocatedRBGs()->size(); alloc_rb_id++) {
          double sinr = amc->GetSinrFromCQI(users->at(uid)->GetCqiFeedbacks().at(alloc_rb_id * rbg_size)); 
          std::cerr << " allocates RBG: " << alloc_rb_id << " sinr:" << sinr << std::endl;
          new_estimatedSinrValues.push_back(sinr);
        }
        int old_estima = EstimateTBSizeByEffSinr(new_estimatedSinrValues, users->at(uid)->GetListOfAllocatedRBGs()->size(), rbg_size);
        double new_sinr = amc->GetSinrFromCQI(crt_rbg_id * rbg_size); 
        new_estimatedSinrValues.push_back(new_sinr);
        int new_estima = EstimateTBSizeByEffSinr(new_estimatedSinrValues, users->at(uid)->GetListOfAllocatedRBGs()->size()+1, rbg_size);
        std::cerr << "  new_estima:" << new_estima << " request:" << pre_defined_gbr_[uid] * 1000 * 1000 / 1000 << std::endl;
        if (new_estima > old_estima) {
          // allocate the RB
          users->at(uid)->GetListOfAllocatedRBGs()->push_back(crt_rbg_id);
          int l = crt_rbg_id * rbg_size, r = (crt_rbg_id + 1) * rbg_size;
          for (int j = l; j < r; ++j) {
            users->at(uid)->GetListOfAllocatedRBs()->push_back(j);
            std::cerr << j << " ";
          }
          rbg_availability[crt_rbg_id] = 0;
          std::cerr << "]" << std::endl;
          if (new_estima > pre_defined_gbr_[uid] * 1000 * 1000 / 1000) {
            satisfied_ues[uid] = 1;
            std::cerr << "  user_id: " << uid << " is satisfied" << std::endl;
          }
        }
       
      }
    }
  }

  // peter: checking how many remain unallocated after all has finished
  int count = 0;
  for (int i = 0; i < nb_rbgs; i++) {
    if (rbg_availability[i] == 1) {
      count++;
    }
  }
  std::cerr << "The count of unallocated RBGs " << count << std::endl;

  // // ========= Allocation for those unallocated RBs: allocate to thsoe unsatisfied UE in a greely way (per RB) =========
  // for those RB that not in any UE's suitable list
  // unallocated RBs
  // for (int i = 0; i < nb_rbgs; i++) {
  //   if (rbg_availability[i] == 1) {
  //     // allocate to the user with the highest CQI
  //     int max_cqi = -1;
  //     int max_cqi_user = -1;
  //     std::cerr << "test ";
  //     for (int ue = 0; ue < users->size(); ue++) {
  //       if (ue_satisfied[ue] == 1) {
  //         continue;
  //       }
  //       std::cerr << ue << " ";
  //       if (max_cqi < users->at(ue)->GetCqiFeedbacks().at(i * rbg_size)) {
  //         std::cerr << "t1 ";
  //         max_cqi = users->at(ue)->GetCqiFeedbacks().at(i * rbg_size);
  //         max_cqi_user = ue;
  //         std::cerr << "t2 ";
  //       }
  //     }
  //     std::cerr << "  Allocation for user_id: " << max_cqi_user << ", rbg_id: " << i << " rb:[";
  //     users->at(max_cqi_user)->GetListOfAllocatedRBGs()->push_back(i);
  //     int l = i * rbg_size, r = (i + 1) * rbg_size;
  //     for (int j = l; j < r; ++j) {
  //       users->at(max_cqi_user)->GetListOfAllocatedRBs()->push_back(j);
  //       std::cerr << j << " ";
  //     }
  //     rbg_availability[i] == 0;
  //     std::cerr << "]" << std::endl;
  //     std::cerr << " Allocation RB:" << i << "  user_id:" << max_cqi_user << " max_cqi:" << max_cqi << std::endl;
  //     // if user is satisfied, then no more allocation
  //     std::vector<double> new_estimatedSinrValues = {};
  //     for (int alloc_rb_id = 0; alloc_rb_id < users->at(max_cqi_user)->GetListOfAllocatedRBGs()->size(); alloc_rb_id++) {
  //       double sinr = amc->GetSinrFromCQI(users->at(max_cqi_user)->GetCqiFeedbacks().at(alloc_rb_id * rbg_size)); 
  //       std::cerr << " allicates RBG: " << alloc_rb_id << " sinr:" << sinr << std::endl;
  //       new_estimatedSinrValues.push_back(sinr);
  //     }
  //     int new_estima = EstimateTBSizeByEffSinr(new_estimatedSinrValues, users->at(max_cqi_user)->GetListOfAllocatedRBGs()->size(), rbg_size);
  //     std::cerr << "  new_estima:" << new_estima << " request:" << pre_defined_gbr_[max_cqi_user] * 1000 * 1000 / 1000 << std::endl;
  //     if (new_estima > pre_defined_gbr_[max_cqi_user] * 1000 * 1000 / 1000) {
  //       ue_satisfied[max_cqi_user] = 1;
  //       std::cerr << "  user_id: " << max_cqi_user << " is satisfied" << std::endl;
  //     }
  //   }
  // }

  // // print those unallocated RB
  // std::cerr << "==== final unallocated RBs ====" << std::endl;
  // for (int i = 0; i < nb_rbgs; i++) {
  //   int rbg_id = rbgid_impact_pair[i].first;
  //   if (rbg_availability[rbg_id] == 1) {
  //     std::cerr << i << "th rbg_id: " << rbgid_impact_pair[i].first << " impact:" << rbgid_impact_pair[i].second;
  //     // print rbg_impact_ues
  //     std::cerr << "  rbg_impact_ues(" << rbg_impact_ues[rbg_id].size() << "): ";
  //     for (int j = 0; j < rbg_impact_ues[rbg_id].size(); j++) {
  //       std::cerr << rbg_impact_ues[rbg_id][j] << " ";
  //     }
  //     std::cerr << std::endl;
  //   }
  // }

  
  PdcchMapIdealControlMessage* pdcchMsg = new PdcchMapIdealControlMessage();
  std::cout << GetTimeStamp() << std::endl;
  for (auto it = users->begin(); it != users->end(); it++) {
    UserToSchedule* ue = *it;

    int ueid = ue->GetUserID();
    std::cerr<< GetTimeStamp() << "ueid: " << ueid << std::endl;

    if (ue->GetListOfAllocatedRBs()->size() > 0) {
      std::vector<double> estimatedSinrValues;

      std::cerr << "For calculation: User " << ue->GetUserID() << std::endl; //" allocated RBGs total_metric_values: ";
      //int total_metric_values = 0;
      for (size_t i = 0; i < ue->GetListOfAllocatedRBs()->size(); i++) {
        int rbid = ue->GetListOfAllocatedRBs()->at(i);
        // if (rbid % 8 == 0){
        //   total_metric_values += metrics[int(rbid / 8)][ueid] * 8;
        //   //std::cerr<< GetTimeStamp() << "Actual Allocate rbid: " << int(rbid / 4) << " metric:" << metrics[rbid][ueid] << std::endl;
        // }

        // fprintf(stderr, "rbid: %d\n", rbid);
        // fprintf(stderr, "ue->GetCqiFeedbacks().size(): %d\n", ue->GetCqiFeedbacks().size());
        assert(rbid < ue->GetCqiFeedbacks().size());

        // if (rbid % rbg_size == 0)
          // std::cerr << " " << rbid / rbg_size << "("
          //           << ue->GetCqiFeedbacks().at(rbid) << ")";


        double sinr = amc->GetSinrFromCQI(
            ue->GetCqiFeedbacks().at(ue->GetListOfAllocatedRBs()->at(i)));
        estimatedSinrValues.push_back(sinr);
      }
      // std::cerr << total_metric_values;

      std::cerr << "finish getting the cqi" << std::endl;
      double effectiveSinr = GetEesmEffectiveSinr(estimatedSinrValues);
      std::cerr << " final_cqi: " << amc->GetCQIFromSinr(effectiveSinr) << std::endl;
      int mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr));
      std::cerr << " mcs:" << mcs << " ue->GetListOfAllocatedRBs()->size():" << ue->GetListOfAllocatedRBs()->size() << std::endl;
      int transportBlockSize =
          amc->GetTBSizeFromMCS(mcs, ue->GetListOfAllocatedRBs()->size());

#if defined(FIRST_SYNTHETIC_EXP) || defined(SECOND_SYNTHETIC_EXP)
      // calculate the transport block size as if the user can have multiple mcs
      transportBlockSize = 0;
      for (int i = 0; i < estimatedSinrValues.size(); i++) {
        transportBlockSize += amc->GetTBSizeFromMCS(
            amc->GetMCSFromCQI(amc->GetCQIFromSinr(estimatedSinrValues[i])), 1);
      }
#endif
      std::cerr << " actual_allocated_bits:" << transportBlockSize << std::endl;

      std::cerr << "User(" << ue->GetUserID() << ") UpdateAllocatedBits:" << transportBlockSize << std::endl;
      ue->UpdateAllocatedBits(transportBlockSize); // unit: bits
      for (size_t rb = 0; rb < ue->GetListOfAllocatedRBs()->size(); rb++) {
        pdcchMsg->AddNewRecord(PdcchMapIdealControlMessage::DOWNLINK,
                               ue->GetListOfAllocatedRBs()->at(rb),
                               ue->GetUserNode(), mcs);
      }
    }
  }
  if (pdcchMsg->GetMessage()->size() > 0) {
    GetMacEntity()->GetDevice()->GetPhy()->SendIdealControlMessage(pdcchMsg);
  }
  delete pdcchMsg;
}


// Jiajin add
// input: delay sensitive slices
// return: rb allocation metric
// vector<int> DownlinkHeterogenousScheduler::RBsAllocation_EDF(int max_num_rbs, UsersToSchedule* users, vector<int> rb_allocation)
// {
  
//   return ;
// }



vector<int> DownlinkHeterogenousScheduler::GetSortedUEsIDbyQoS(map<int, double> user_qos_map, std::vector<std::deque<double>>& allocation_logs, double threshold, int total_rbgs_to_allocate) {

    // Peter: calculate the sum of the allocation logs
    vector<double> allocate_sum_hist;
    allocate_sum_hist.resize(num_windows_);
    std::fill(allocate_sum_hist.begin(), allocate_sum_hist.end(), 0);
    for (int i = 0; i < num_windows_; i++) {
      for (int j = 0; j < allocation_logs[i].size(); j++) {
        allocate_sum_hist[i] += allocation_logs[i][j];
      }
    }

    // peter: caluclate the requested rate of each user


    // Convert map to a vector of pairs for sorting
    vector<pair<int, double>> user_delay_pair(user_qos_map.begin(), user_qos_map.end());

    // peter: filtered user delay pair
    vector<pair<int, double>> filtered_user_delay_pair;
 
    // peter: filter out those users whose allocation is less than the threshold
    for (const auto &pair : user_delay_pair) {
      int user_id = pair.first;
      double request_rate = WINDOW_SIZE * pair.second - allocate_sum_hist[user_id];
      if (request_rate >= threshold) {
        filtered_user_delay_pair.push_back(std::make_pair(user_id, request_rate));
      } else {
        fprintf(stderr, "User %d requesting rate: %f is less than the threshold: %f\n", user_id, request_rate, threshold);
      }
    }

    fprintf(stderr, "filtered_user_delay_pair.size(): %d\n", filtered_user_delay_pair.size());

    // Sort the vector by increasing order of its pair's second value
    // TODO: try GBR from max to min later
    sort(filtered_user_delay_pair.begin(), filtered_user_delay_pair.end(), sortByVal);

    for (int i = 0; i < filtered_user_delay_pair.size(); i++) {
      fprintf(stderr, "user_id: %d, request_rate: %f\n", filtered_user_delay_pair[i].first, filtered_user_delay_pair[i].second);
    }

    // Extract sorted user IDs from sorted pair vector
    vector<int> sorted_selected_users_ids;
    for (const auto &pair : filtered_user_delay_pair) {
        sorted_selected_users_ids.push_back(pair.first);
    }

    // to accomodate for the situation where there is a surplus of rbgs after the filtering operation
    if (sorted_selected_users_ids.size() < total_rbgs_to_allocate) {
      fprintf(stderr, "Warning: the number of users requesting rate larger than the threshold is less than the number of rbgs to allocate\n");
      sort(user_delay_pair.begin(), user_delay_pair.end(), sortByValDesc);
      for (int j = 0; j < total_rbgs_to_allocate - sorted_selected_users_ids.size(); j++) {
        sorted_selected_users_ids.push_back(user_delay_pair[j].first);
      }
    }

    return sorted_selected_users_ids;
}

// Jiajin NOTE: actually unused
double DownlinkHeterogenousScheduler::ComputeSchedulingMetric(
    UserToSchedule* user, double spectralEfficiency) {
  double metric = 0;
  double averageRate = 1;
  int slice_id = user_to_slice_[user->GetUserID()];
  for (int i = 0; i < MAX_BEARERS; ++i) {
    if (user->m_bearers[i]) {
      averageRate += user->m_bearers[i]->GetAverageTransmissionRate();
    }
  }
  spectralEfficiency =
      spectralEfficiency * 180000 / 1000;  // set the unit to kbps
  averageRate /= 1000.0;  // set the unit of both params to kbps
  SchedulerAlgoParam param = slice_algo_params_[slice_id];
  if (param.alpha == 0) {
    metric =
        pow(spectralEfficiency, param.epsilon) / pow(averageRate, param.psi);
  } else {
    // the prioritized flow has no packet, set metric to 0
    if (user->m_dataToTransmit[slice_priority_[slice_id]] == 0) {
      metric = 0;
    } else {
      RadioBearer* bearer = user->m_bearers[slice_priority_[slice_id]];
      if (param.beta) {
        double HoL = bearer->GetHeadOfLinePacketDelay();
        metric = -HoL * pow(spectralEfficiency, param.epsilon) /
                 pow(averageRate, param.psi);
      } else {
        metric = pow(spectralEfficiency, param.epsilon) /
                 pow(averageRate, param.psi);
      }
    }
  }
  return metric;
}