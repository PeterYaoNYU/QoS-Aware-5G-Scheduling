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
      // slice_algo_params_.emplace_back(slice_schemes[i]["type"].asInt());
      // fprintf(stderr, "slice %d, weight %f, type %d\n", i, slice_schemes[i]["weight"].asDouble(), slice_schemes[i]["type"].asInt());
      // Jiajin remove

      // fprintf(stderr, "slice %d, weight %f, type %d\n", i, slice_schemes[i]["weight"].asDouble(), slice_schemes[i]["type"].asInt());

      // Peter Remark: I think this should be the right place to place 
      // bit rate / delay / type of the UE etc. If such parameters come along with the Slice instead of the UE
    }
  }
  // [peter] for each slice, calculate the priority
  slice_priority_.resize(num_slices_);
  std::fill(slice_priority_.begin(), slice_priority_.end(), 0);
  // [Peter] used later to ensure that the allocations of rbgs overtime is overall fair and accords to the SLA
  // Jiajin remove
  // slice_rbs_offset_.resize(num_slices_); 
  // std::fill(slice_rbs_offset_.begin(), slice_rbs_offset_.end(), 0);

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
    slice_target_rbs[slice_id] = (int)(nb_rbs * slice_weights_[slice_id]);
        //(int)(nb_rbs * slice_weights_[slice_id] + slice_rbs_offset_[slice_id]); // Jiajin remove
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
  // peter: If there are leftover RBs after the even distribution, allocate them to the first slice
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
  
  // ========== Metric Calculation ==========
  // the metric here is based on the available data rate if UE_i using RB_j
  // TODO: only calculate for those UE candidates (Top K UEs in each slice)
  double metrics[nb_rbgs][users->size()];
  // also calculate the total spectrum efficiency of the RB for all UEs
  double rb_metric_sum[nb_rbgs] = {}; // Initializing array elements to 0

  // Peter: init to 0 for the array rb_metric_sum
  for (int i = 0; i < nb_rbgs; i++) {
    rb_metric_sum[i] = 0;
  }

  for (int i = 0; i < nb_rbgs; i++) {
    for (size_t j = 0; j < users->size(); j++) {
      metrics[i][j] = users->at(j)->GetSpectralEfficiency().at(i * rbg_size) * 180000 / 1000;
      rb_metric_sum[i] += metrics[i][j];
    }
  }
  std::cerr << GetTimeStamp() << " metrics: ";
  for (int i = 0; i < nb_rbgs; i++) {
    std::cerr << std::endl;
    std::cerr << "rb_metric_sum[" << i << ": " << rb_metric_sum[i] << "    ";
    for (size_t j = 0; j < users->size(); j++) {
      std::cerr << "(" << i << ", " << j << ", " << metrics[i][j] << ") ";
    }
  }
  
  std::vector<int> available_rbs(nb_rbgs, -1); // Initialization with -1: rb_allocation[i] = UE_id
  for (int i = 0; i < nb_rbgs; i++)
  {
    available_rbs.push_back(i);
  }

  // std::cerr << GetTimeStamp() << " available_rbs: ";
  // for (int i = 0; i < available_rbs.size(); i++) {
  //   std::cerr << available_rbs[i] << " ";
  // }
  // std::cerr << std::endl;
  
  // ========== Sort UEs by Min Request Rate First ==========
  map<int, double> user_request_map; // user_id -> request_rate
  // map<int, double> user_delay_map;
  vector<int> user_be; // preparation for BE
  // Calculate required rate: just like ====D3====
  for (auto it = users->begin(); it != users->end(); ++it) {
    int user_id = (*it)->GetUserID();
    int slice_id = user_to_slice_[user_id];
    if (slice_algo_params_[slice_id].type == 1) // filter those delay-sensitive UEs
    {
      assert(slice_algo_params_[slice_id].type == 1);
      RadioBearer* bearer = (*it)->m_bearers[0]; // TODO: currently assume only one bearer for each user (originally is slice_priority_[slice_id])
      QoSParameters* qos = bearer->GetQoSParameters();
      double delay = qos->GetMaxDelay(); // TODO: floor the maxdelay. E.g: 5.4ms should be 5ms (assume TTI is 1ms)
      //user_delay_map.insert(make_pair(user_id, delay));
      // TODO: consider only the first pkt in the queue
      //double packet_size = bearer->GetMacQueue()->Peek().GetSize(); // To Check: whether the pkt is a whole
      int accumlated_bytes = bearer->GetMacQueue()->GetQueueSize();
      user_request_map.insert(make_pair(user_id, accumlated_bytes / delay));
      std::cerr << GetTimeStamp() << " user_id: " << user_id << " accumlated_bytes: " << accumlated_bytes << " delay: " << delay << " request_rate: " << accumlated_bytes / delay << std::endl;
    }
    else if (slice_algo_params_[slice_id].type == 2) // filter those GBR UEs
    {
      assert(slice_algo_params_[slice_id].type == 2);
      RadioBearer* bearer = (*it)->m_bearers[0]; // TODO: currently assume only one bearer for each user (originally is slice_priority_[slice_id])
      QoSParameters* qos = bearer->GetQoSParameters();
      double gbr = qos->GetGBR(); // TODO: should add maxdelay in pkt's qosparamaters: best-effort should be -1
      user_request_map.insert(make_pair(user_id, gbr));
      std::cerr << GetTimeStamp() << " user_id: " << user_id << " request_rate: " << gbr << std::endl;
    }
    else{
      user_be.push_back(user_id);
      std::cerr << GetTimeStamp() << " user_id: " << user_id << " best effort" << std::endl;
    }
  } 

  vector<int> request_users = GetSortedUEsIDbyQoS(user_request_map); // TODO: maybe later, each slice can decide its own serving order of UEs


  // ========== RB Allocation ==========
  vector<int> slice_allocated_rbs(num_slices_, 0);
  vector<int> slice_satisifed_ue_num(num_slices_, 0);
  vector<int> unsatisfied_users; // those users who cannot be served due to RBs shortage, but can be stored and served later if RBs are available
  for (int i = 0; i < request_users.size(); i++)
  {
    int user_id = request_users[i];
    int slice_id = user_to_slice_[user_id];
    // if slice reaches its weight quota
    if(slice_allocated_rbs[slice_id] >= slice_quota_rbgs[slice_id]){
      unsatisfied_users.push_back(user_id);
      continue;
    }
    // rb allocation based on request
    double request_rate = user_request_map[user_id];
    while (request_rate > 0 && slice_allocated_rbs[slice_id] < slice_quota_rbgs[slice_id])
    {
      // find those situtable RBs
      int target_rb_id = -1;
      int max_rate_rb_id = -1;
      double min_residual_rate = std::numeric_limits<double>::max();
      double max_rate = 0;
      for (int j = 0; j < available_rbs.size(); j++)
      {
        int rb_id = available_rbs[j];
        if (metrics[rb_id][user_id] > request_rate && rb_metric_sum[rb_id] - mmax_residual_rateetrics[rb_id][user_id] < min_residual_rate)
        {
          target_rb_id = rb_id;
          min_residual_rate = rb_metric_sum[rb_id] - metrics[rb_id][user_id]; // TODO: Currently, it is an intuitive heuristic fucntion, but later, other methods should also be considered
        }
        if (target_rb_id == -1 && metrics[rb_id][user_id] > max_rate)
        {
          max_rate = metrics[rb_id][user_id];
          max_rate_rb_id = rb_id;
        }
      }
      if (target_rb_id == -1)
      {
        target_rb_id = max_rate_rb_id;
      }
      request_rate -= metrics[target_rb_id][user_id];
      int l = target_rb_id * rbg_size, r = (target_rb_id + 1) * rbg_size;
      for (int j = l; j < r; ++j) {
        users->at(user_id)->GetListOfAllocatedRBs()->push_back(j);
      }
      // erase the allocated RBs
      available_rbs.erase(remove(available_rbs.begin(), available_rbs.end(), target_rb_id), available_rbs.end());
      slice_allocated_rbs[slice_id] += 1;
      // TODO: if there is no more UEs to be served in the slice, the weight can be shared by UEs within the slice or other slices 
      // if (request_rate <= 0)
      // {
      //   break;
      // }
    }
  } 
  // for those unsatisfied in the previous stage, try to allocate RBs to them. TODO: may do not need it, becasue these RBs can also be shared by UEs in their slice.

  
  // == TODO: Lastly, Best-effort (MT or PF)

  // TODO[suggested by copilot]: free the space of the users to schedule

  PdcchMapIdealControlMessage* pdcchMsg = new PdcchMapIdealControlMessage();
  std::cout << GetTimeStamp() << std::endl;
  for (auto it = users->begin(); it != users->end(); it++) {
    UserToSchedule* ue = *it;
    if (ue->GetListOfAllocatedRBs()->size() > 0) {
      std::vector<double> estimatedSinrValues;

      std::cout << "User(" << ue->GetUserID() << ") allocated RBGS:";
      for (size_t i = 0; i < ue->GetListOfAllocatedRBs()->size(); i++) {
        int rbid = ue->GetListOfAllocatedRBs()->at(i);
        if (rbid % rbg_size == 0)
          std::cout << " " << rbid / rbg_size << "("
                    << ue->GetCqiFeedbacks().at(rbid) << ")";

        double sinr = amc->GetSinrFromCQI(
            ue->GetCqiFeedbacks().at(ue->GetListOfAllocatedRBs()->at(i)));
        estimatedSinrValues.push_back(sinr);
      }
      double effectiveSinr = GetEesmEffectiveSinr(estimatedSinrValues);
      std::cout << " final_cqi: " << amc->GetCQIFromSinr(effectiveSinr)
                << std::endl;
      int mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr));
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
      ue->UpdateAllocatedBits(transportBlockSize);
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

// Jiajin add
// input: all users to be scheduled
// return: sorted list of delay-sensitive users by increasing DDL
bool sortByVal(const std::pair<int, double> &a, const std::pair<int, double> &b) {
    return a.second < b.second;
}

vector<int> DownlinkHeterogenousScheduler::GetSortedUEsIDbyQoS(map<int, double> user_qos_map) {
    // Convert map to a vector of pairs for sorting
    vector<pair<int, double>> user_delay_pair(user_qos_map.begin(), user_qos_map.end());

    // Sort the vector by increasing order of its pair's second value
    // TODO: try GBR from max to min later
    sort(user_delay_pair.begin(), user_delay_pair.end(), sortByVal);

    // Extract sorted user IDs from sorted pair vector
    vector<int> sorted_selected_users_ids;
    for (const auto &pair : user_delay_pair) {
        sorted_selected_users_ids.push_back(pair.first);
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