#include "TSU_SimpleFLIN.h"
#include "NVM_PHY_ONFI_NVDDR2.h"
#include <stack>
#include <cmath>

namespace SSD_Components
{
	TSU_SIMPLE_FLIN::TSU_SIMPLE_FLIN(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI_NVDDR2* NVMController,
		const unsigned int channel_count, const unsigned int chip_no_per_channel, const unsigned int die_no_per_chip, const unsigned int plane_no_per_die, unsigned int flash_page_size,
		const sim_time_type flow_classification_epoch, const unsigned int alpha_read, const unsigned int alpha_write,
		const stream_id_type max_flow_id, const double f_thr, const unsigned int GC_FLIN,
		const sim_time_type WriteReasonableSuspensionTimeForRead,
		const sim_time_type EraseReasonableSuspensionTimeForRead,
		const sim_time_type EraseReasonableSuspensionTimeForWrite,
		const bool EraseSuspensionEnabled, const bool ProgramSuspensionEnabled)
		: TSU_Base(id, ftl, NVMController, Flash_Scheduling_Type::SIMPLE_FLIN, channel_count, chip_no_per_channel, die_no_per_chip, plane_no_per_die,
			max_flow_id, WriteReasonableSuspensionTimeForRead, EraseReasonableSuspensionTimeForRead, EraseReasonableSuspensionTimeForWrite,
			EraseSuspensionEnabled, ProgramSuspensionEnabled),
		flow_classification_epoch(flow_classification_epoch),F_thr(f_thr), GC_flin(GC_FLIN)
	{
		alpha_read_for_epoch = alpha_read / (channel_count * chip_no_per_channel) / flash_page_size;
		alpha_write_for_epoch = alpha_write / (channel_count * chip_no_per_channel) / flash_page_size;
		UserReadTRQueue = new Flash_Transaction_Queue * [channel_count];
		UserWriteTRQueue = new Flash_Transaction_Queue * [channel_count];
		GCReadTRQueue = new Flash_Transaction_Queue * [channel_count];
		GCWriteTRQueue = new Flash_Transaction_Queue * [channel_count];
		GCEraseTRQueue = new Flash_Transaction_Queue * [channel_count];
		MappingReadTRQueue = new Flash_Transaction_Queue * [channel_count];
		MappingWriteTRQueue = new Flash_Transaction_Queue * [channel_count];
		flow_activity_info = new Simple_FLIN_Flow_Monitoring_Unit * *[channel_count];
		low_intensity_class_read = new std::set<stream_id_type> *[channel_count];
		low_intensity_class_write = new std::set<stream_id_type> *[channel_count];
		head_high_read = new std::list<NVM_Transaction_Flash*>::iterator * [channel_count];
		head_high_write = new std::list<NVM_Transaction_Flash*>::iterator * [channel_count];
		transaction_waiting_dispatch_slots = new std::list<std::pair<Transaction_Type, Transaction_Source_Type>> *[channel_count];
		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
		{
			UserReadTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			UserWriteTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			GCReadTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			GCWriteTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			GCEraseTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			MappingReadTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			MappingWriteTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			flow_activity_info[channel_id] = new Simple_FLIN_Flow_Monitoring_Unit * [chip_no_per_channel];
			low_intensity_class_read[channel_id] = new std::set<stream_id_type>[chip_no_per_channel];
			low_intensity_class_write[channel_id] = new std::set<stream_id_type>[chip_no_per_channel];
			head_high_read[channel_id] = new std::list<NVM_Transaction_Flash*>::iterator[chip_no_per_channel];
			head_high_write[channel_id] = new std::list<NVM_Transaction_Flash*>::iterator[chip_no_per_channel];
			transaction_waiting_dispatch_slots[channel_id] = new std::list<std::pair<Transaction_Type, Transaction_Source_Type>>[chip_no_per_channel];
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++)
			{
				flow_activity_info[channel_id][chip_id] = new Simple_FLIN_Flow_Monitoring_Unit[stream_count];
				head_high_read[channel_id][chip_id] = UserReadTRQueue[channel_id][chip_id].end();
				head_high_write[channel_id][chip_id] = UserWriteTRQueue[channel_id][chip_id].end();

				UserReadTRQueue[channel_id][chip_id].Set_id("User_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				UserWriteTRQueue[channel_id][chip_id].Set_id("User_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				GCReadTRQueue[channel_id][chip_id].Set_id("GC_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id) + "@");
				MappingReadTRQueue[channel_id][chip_id].Set_id("Mapping_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				MappingWriteTRQueue[channel_id][chip_id].Set_id("Mapping_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				GCWriteTRQueue[channel_id][chip_id].Set_id("GC_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				GCEraseTRQueue[channel_id][chip_id].Set_id("GC_Erase_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));

				for (unsigned int stream_cntr = 0; stream_cntr < stream_count; stream_cntr++)
				{
					low_intensity_class_read[channel_id][chip_id].insert(stream_cntr);
					low_intensity_class_write[channel_id][chip_id].insert(stream_cntr);
				}
			}
		}
	}

	TSU_SIMPLE_FLIN::~TSU_SIMPLE_FLIN()
	{
		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
		{
			delete[] UserReadTRQueue[channel_id];
			delete[] UserWriteTRQueue[channel_id];
			delete[] GCReadTRQueue[channel_id];
			delete[] GCWriteTRQueue[channel_id];
			delete[] GCEraseTRQueue[channel_id];
			delete[] MappingReadTRQueue[channel_id];
			delete[] MappingWriteTRQueue[channel_id];
			delete[] low_intensity_class_read[channel_id];
			delete[] low_intensity_class_write[channel_id];
			delete[] head_high_read[channel_id];
			delete[] head_high_write[channel_id];
			delete[] transaction_waiting_dispatch_slots[channel_id];
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++)
			{
				delete[] flow_activity_info[channel_id][chip_id];
			}
			delete[] flow_activity_info[channel_id];
		}
		delete[] UserReadTRQueue;
		delete[] UserWriteTRQueue;
		delete[] GCReadTRQueue;
		delete[] GCWriteTRQueue;
		delete[] GCEraseTRQueue;
		delete[] MappingReadTRQueue;
		delete[] MappingWriteTRQueue;
		delete[] low_intensity_class_read;
		delete[] low_intensity_class_write;
		delete[] head_high_read;
		delete[] head_high_write;
		delete[] flow_activity_info;
		delete[] transaction_waiting_dispatch_slots;
	}

	void TSU_SIMPLE_FLIN::Start_simulation()
	{
		Simulator->Register_sim_event(flow_classification_epoch, this, 0, 0);
	}

	void TSU_SIMPLE_FLIN::Validate_simulation_config() {}

	void TSU_SIMPLE_FLIN::Execute_simulator_event(MQSimEngine::Sim_Event* event)
	{
		//Flow classification as described in Section 5.1 of FLIN paper in ISCA 2018
		if (stream_count > 1)
		{
			for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
			{
				for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++)
				{
					low_intensity_class_read[channel_id][chip_id].clear();
					low_intensity_class_write[channel_id][chip_id].clear();
					for (unsigned int stream_cntr = 0; stream_cntr < stream_count; stream_cntr++)
					{
						if (flow_activity_info[channel_id][chip_id][stream_cntr].Serviced_read_requests_recent < alpha_read_for_epoch)
							low_intensity_class_read[channel_id][chip_id].insert(stream_cntr);
						if (flow_activity_info[channel_id][chip_id][stream_cntr].Serviced_write_requests_recent < alpha_write_for_epoch)
							low_intensity_class_write[channel_id][chip_id].insert(stream_cntr);
						flow_activity_info[channel_id][chip_id][stream_cntr].Serviced_read_requests_recent = 0;
						flow_activity_info[channel_id][chip_id][stream_cntr].Serviced_write_requests_recent = 0;
					}
				}
			}
		}
		Simulator->Register_sim_event(Simulator->Time() + flow_classification_epoch, this, 0, 0);
	}

	size_t TSU_SIMPLE_FLIN::GCEraseTRQueueSize(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id)
	{
		return GCEraseTRQueue[channel_id][chip_id].size() + GCWriteTRQueue[channel_id][chip_id].size() + GCReadTRQueue[channel_id][chip_id].size();
	}

	size_t TSU_SIMPLE_FLIN::UserWriteTRQueueSize(stream_id_type gc_stream_id, flash_channel_ID_type channel_id, flash_chip_ID_type chip_id)
	{
		size_t cnt = 0;
		for (auto it_tr : UserWriteTRQueue[channel_id][chip_id])
		{
			if (it_tr->Stream_id == gc_stream_id) ++cnt;
		}
		for (auto it_tr : UserReadTRQueue[channel_id][chip_id])
		{
			if (it_tr->Stream_id == gc_stream_id) ++cnt;
		}
		return cnt;
	}

	inline void TSU_SIMPLE_FLIN::Prepare_for_transaction_submit()
	{
		opened_scheduling_reqs++;
		if (opened_scheduling_reqs > 1)
			return;
		transaction_receive_slots.clear();
	}

	inline void TSU_SIMPLE_FLIN::Submit_transaction(NVM_Transaction_Flash* transaction)
	{
		transaction_receive_slots.push_back(transaction);
	}

	void TSU_SIMPLE_FLIN::Schedule()
	{
		opened_scheduling_reqs--;
		if (opened_scheduling_reqs > 0)
			return;
		if (opened_scheduling_reqs < 0)
			PRINT_ERROR("TSU Schedule function is invoked in an incorrect way!");

		if (transaction_receive_slots.size() == 0)
			return;


		for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_receive_slots.begin();
			it != transaction_receive_slots.end(); it++)
		{
			flash_channel_ID_type channel_id = (*it)->Address.ChannelID;
			flash_chip_ID_type chip_id = (*it)->Address.ChipID;
			stream_id_type stream_id = (*it)->Stream_id;
			(*it)->is_conflicting_gc = _NVMController->Is_chip_busy_with_gc((*it)->Address.ChannelID, (*it)->Address.ChipID);
			switch ((*it)->Type)
			{
			case Transaction_Type::READ:
				switch ((*it)->Source)
				{
				case Transaction_Source_Type::CACHE:
				case Transaction_Source_Type::USERIO:
					estimate_alone_time(*it, &UserReadTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID]);
					if (stream_count < 2)
					{
						UserReadTRQueue[channel_id][chip_id].push_back(*it);
						break;
					}
					flow_activity_info[channel_id][chip_id][stream_id].Serviced_read_requests_recent++;
					if (low_intensity_class_read[channel_id][chip_id].find(stream_id) == low_intensity_class_read[channel_id][chip_id].end())
					{
						UserReadTRQueue[channel_id][chip_id].push_back(*it);//Insert(TRnew after Q.Tail)
						auto tail = UserReadTRQueue[channel_id][chip_id].end();
						tail--;
						auto copy_tail = tail;
						if (head_high_read[channel_id][chip_id] == UserReadTRQueue[channel_id][chip_id].end())
						{
							head_high_read[channel_id][chip_id] = tail;
						}
						estimate_alone_waiting_time(&UserReadTRQueue[channel_id][chip_id], tail);//EstimateAloneWaitingTime(Q, TRnew)
						stream_id_type flow_with_max_average_slowdown;
						double f = fairness_based_on_average_slowdown(&UserReadTRQueue[channel_id][chip_id],
							head_high_read[channel_id][chip_id], flow_with_max_average_slowdown);//FairnessBasedOnAverageSlowdown(Q)
						if (f < F_thr && stream_id == flow_with_max_average_slowdown)
							move_forward(&UserReadTRQueue[channel_id][chip_id],
								tail,
								head_high_read[channel_id][chip_id]);//MoveForward(from Q.Tail up to Q.Taillow + 1)
						else reorder_for_fairness(&UserReadTRQueue[channel_id][chip_id],
							head_high_read[channel_id][chip_id], tail);//ReorderForFairness(from Q.Tail to Q.Taillow + 1)
					}
					else
					{
						UserReadTRQueue[channel_id][chip_id].insert(head_high_read[channel_id][chip_id], *it);//Insert(TRnew after Q.Taillow)
						auto it_tr = head_high_read[channel_id][chip_id];
						while (it_tr != UserReadTRQueue[channel_id][chip_id].end())
						{
							move_alone_time(*it, *it_tr);
							++it_tr;
						}
						auto tail_low = head_high_read[channel_id][chip_id];
						if (tail_low != UserReadTRQueue[channel_id][chip_id].begin())
							tail_low--;
						estimate_alone_waiting_time(&UserReadTRQueue[channel_id][chip_id], tail_low);//EstimateAloneWaitingTime(Q, TRnew)
						reorder_for_fairness(&UserReadTRQueue[channel_id][chip_id],
							UserReadTRQueue[channel_id][chip_id].begin(), tail_low);//ReorderForFairness(from Q.Taillow to Q.Head)
					}
					break;
				case Transaction_Source_Type::MAPPING:
					MappingReadTRQueue[channel_id][chip_id].push_back(*it);
					break;
				case Transaction_Source_Type::GC_WL:
					GCReadTRQueue[channel_id][chip_id].push_back(*it);
					break;
				default:
					PRINT_ERROR("TSU_FLIN: Unhandled source type four read transaction!")
				}
				break;
			case Transaction_Type::WRITE:
				switch ((*it)->Source)
				{
				case Transaction_Source_Type::CACHE:
				case Transaction_Source_Type::USERIO:
					estimate_alone_time(*it, &UserWriteTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID]);
					if (stream_count < 2)
					{
						UserWriteTRQueue[channel_id][chip_id].push_back(*it);
						break;
					}
					flow_activity_info[channel_id][chip_id][stream_id].Serviced_write_requests_recent++;
					if (low_intensity_class_write[channel_id][chip_id].find(stream_id) == low_intensity_class_write[channel_id][chip_id].end())
					{
						UserWriteTRQueue[channel_id][chip_id].push_back(*it);//Insert(TRnew after Q.Tail)
						auto tail = UserWriteTRQueue[channel_id][chip_id].end();
						tail--;
						if (head_high_write[channel_id][chip_id] == UserWriteTRQueue[channel_id][chip_id].end())
						{
							head_high_write[channel_id][chip_id] = tail;
						}
						estimate_alone_waiting_time(&UserWriteTRQueue[channel_id][chip_id], tail);//EstimateAloneWaitingTime(Q, TRnew)
						stream_id_type flow_with_max_average_slowdown;
						double f = fairness_based_on_average_slowdown(&UserWriteTRQueue[channel_id][chip_id],
							head_high_write[channel_id][chip_id], flow_with_max_average_slowdown);//FairnessBasedOnAverageSlowdown(Q)
						if (f < F_thr && stream_id == flow_with_max_average_slowdown)
							move_forward(&UserWriteTRQueue[channel_id][chip_id],
								tail,
								head_high_write[channel_id][chip_id]);//MoveForward(from Q.Tail up to Q.Taillow + 1)
						else reorder_for_fairness(&UserWriteTRQueue[channel_id][chip_id],
							head_high_write[channel_id][chip_id], tail);//ReorderForFairness(from Q.Tail to Q.Taillow + 1)
					}
					else
					{
						auto it_tr = head_high_write[channel_id][chip_id];
						while (it_tr != UserWriteTRQueue[channel_id][chip_id].end())
						{
							move_alone_time(*it, *it_tr);
							++it_tr;
						}
						UserWriteTRQueue[channel_id][chip_id].insert(head_high_write[channel_id][chip_id], *it);
						auto tail_low = head_high_write[channel_id][chip_id];
						if (tail_low != UserWriteTRQueue[channel_id][chip_id].begin())
							tail_low--;
						estimate_alone_waiting_time(&UserWriteTRQueue[channel_id][chip_id], tail_low);
						reorder_for_fairness(&UserWriteTRQueue[channel_id][chip_id],
							UserWriteTRQueue[channel_id][chip_id]	.begin(), tail_low);
					}
					break;
				case Transaction_Source_Type::MAPPING:
					MappingWriteTRQueue[channel_id][chip_id].push_back(*it);
					break;
				case Transaction_Source_Type::GC_WL:
					GCWriteTRQueue[channel_id][chip_id].push_back(*it);
					break;
				default:
					PRINT_ERROR("TSU_FLIN: Unhandled source type four write transaction!")
				}
				break;
			case Transaction_Type::ERASE:
				GCEraseTRQueue[channel_id][chip_id].push_back(*it);
				break;
			default:
				break;
			}
		}


		for (flash_channel_ID_type channel_id = 0; channel_id < channel_count; channel_id++)
		{
			if (_NVMController->Get_channel_status(channel_id) == BusChannelStatus::IDLE)
			{
				for (unsigned int i = 0; i < chip_no_per_channel; i++) {
					NVM::FlashMemory::Flash_Chip* chip = _NVMController->Get_chip(channel_id, Round_robin_turn_of_channel[channel_id]);
					//The TSU does not check if the chip is idle or not since it is possible to suspend a busy chip and issue a new command
					service_transaction(chip);
					Round_robin_turn_of_channel[channel_id] = (flash_chip_ID_type)(Round_robin_turn_of_channel[channel_id] + 1) % chip_no_per_channel;
					if (_NVMController->Get_channel_status(chip->ChannelID) != BusChannelStatus::IDLE)
						break;
				}
			}
		}
	}

	void TSU_SIMPLE_FLIN::reorder_for_fairness(Flash_Transaction_Queue* queue, std::list<NVM_Transaction_Flash*>::iterator start,
		std::list<NVM_Transaction_Flash*>::iterator end)
	{
		if (start == end)
		{
			return;
		}
		std::list<NVM_Transaction_Flash*>::iterator itr = queue->begin();
		// add
		std::unordered_map<stream_id_type, std::pair<double, unsigned int>> slowdown_info_per_workload;
		// end add
		sim_time_type time_to_finish = 0;
		if (_NVMController->Is_chip_busy(*itr))
			if (_NVMController->Expected_finish_time(*itr) > Simulator->Time())
				time_to_finish = _NVMController->Expected_finish_time(*itr) - Simulator->Time();//T^chip_busy

		while (itr != start)
		{
			time_to_finish += _NVMController->Expected_transfer_time(*itr) + _NVMController->Expected_command_time(*itr);
			itr++;
		}

		//First pass: from start to end to estimate the slowdown of each transaction in its current position
		std::stack<double> min_slowdown_list, max_slowdown_list;
		double slowdown_min = DBL_MAX, slowdown_max = 0;
		while (itr != std::next(end))
		{
			time_to_finish += _NVMController->Expected_transfer_time(*itr) + _NVMController->Expected_command_time(*itr);
			sim_time_type T_TR_shared = time_to_finish + (Simulator->Time() - (*itr)->Issue_time);
			double slowdown = (double)T_TR_shared / (double)((*itr)->Estimated_alone_waiting_time
				+ _NVMController->Expected_transfer_time(*itr) + _NVMController->Expected_command_time(*itr));
			// add
			if (!slowdown_info_per_workload.count((*itr)->Stream_id))
			{
				slowdown_info_per_workload[(*itr)->Stream_id].first = 0.0;
				slowdown_info_per_workload[(*itr)->Stream_id].second = 0;
			}
			slowdown_info_per_workload[(*itr)->Stream_id].first += slowdown;
			slowdown_info_per_workload[(*itr)->Stream_id].second += 1;
			// end add
			if (slowdown < slowdown_min)
				slowdown_min = slowdown;
			if (slowdown > slowdown_max)
				slowdown_max = slowdown;
			min_slowdown_list.push(slowdown_min);
			max_slowdown_list.push(slowdown_max);
			itr++;
		}
		double fairness_max = slowdown_min / slowdown_max;

		// add
		double slowdown_min_workload = DBL_MAX, slowdown_max_workload = 0;
		for (const auto& a : slowdown_info_per_workload)
		{
			double slowdown = a.second.first / a.second.second;
			if (slowdown < slowdown_min_workload)
				slowdown_min_workload = slowdown;
			if (slowdown > slowdown_max_workload)
				slowdown_max_workload = slowdown;
		}
		fairness_max = slowdown_min_workload / slowdown_max_workload;
		// end add

		//Second pass: from end to start to find a position for TR_new sitting at end, that maximizes fairness
		sim_time_type T_new_alone = (*end)->Estimated_alone_waiting_time
			+ _NVMController->Expected_transfer_time(*end) + _NVMController->Expected_command_time(*end);

		auto final_position = end;
		auto traverser = std::prev(end);
		time_to_finish -= _NVMController->Expected_transfer_time(*end) + _NVMController->Expected_command_time(*end);

		while (traverser != std::prev(start) && (*traverser)->Stream_id != (*end)->Stream_id)
		{
			sim_time_type T_pos_alone = (*traverser)->Estimated_alone_waiting_time
				+ _NVMController->Expected_transfer_time(*traverser) + _NVMController->Expected_command_time(*traverser);

			sim_time_type T_pos_shared_after = time_to_finish + _NVMController->Expected_transfer_time(*end) + _NVMController->Expected_command_time(*end)
				+ (Simulator->Time() - (*traverser)->Issue_time);
			double slowdown_pos_after = (double)T_pos_shared_after / T_pos_alone;

			sim_time_type T_new_shared_after = time_to_finish + (Simulator->Time() - (*end)->Issue_time)
				- _NVMController->Expected_transfer_time(*traverser) - _NVMController->Expected_command_time(*traverser)
				+ _NVMController->Expected_transfer_time(*end) + _NVMController->Expected_command_time(*end);
			double slowdown_new_after = (double)T_new_shared_after / T_new_alone;

			// add
			sim_time_type T_pos_shared_before = time_to_finish + (Simulator->Time() - (*traverser)->Issue_time);
			double slowdown_pos_before = (double)T_pos_shared_before / T_pos_alone;
			slowdown_info_per_workload[(*traverser)->Stream_id].first += slowdown_pos_after - slowdown_pos_before;
			sim_time_type T_new_shared_before = time_to_finish + (Simulator->Time() - (*end)->Issue_time)
				+ _NVMController->Expected_transfer_time(*end) + _NVMController->Expected_command_time(*end);
			double slowdown_new_before = (double)T_new_shared_before / T_new_alone;
			slowdown_info_per_workload[(*end)->Stream_id].first += slowdown_new_after - slowdown_new_before;
			// end add

			double slowdown_min = min_slowdown_list.top();
			min_slowdown_list.pop();
			double slowdown_max = max_slowdown_list.top();
			max_slowdown_list.pop();
			if (slowdown_pos_after > slowdown_max)
				slowdown_max = slowdown_pos_after;
			if (slowdown_pos_after < slowdown_min)
				slowdown_min = slowdown_pos_after;

			if (slowdown_new_after > slowdown_max)
				slowdown_max = slowdown_new_after;
			if (slowdown_new_after < slowdown_min)
				slowdown_min = slowdown_new_after;

			double fairness_after = (double)slowdown_min / slowdown_max;
			// add
			double slowdown_min_workload = DBL_MAX, slowdown_max_workload = 0;
			for (const auto& a : slowdown_info_per_workload)
			{
				double slowdown = a.second.first / a.second.second;
				if (slowdown < slowdown_min_workload)
					slowdown_min_workload = slowdown;
				if (slowdown > slowdown_max_workload)
					slowdown_max_workload = slowdown;
			}
			fairness_after = slowdown_min_workload / slowdown_max_workload;
			// end add
			if (fairness_after > fairness_max)
			{
				fairness_max = fairness_after;
				final_position = traverser;
			}

			time_to_finish -= _NVMController->Expected_transfer_time(*traverser) + _NVMController->Expected_command_time(*traverser);
			--traverser;
		}
		auto it_tr = final_position;
		while (it_tr != end)
		{
			move_alone_time(*end, *it_tr);
			++it_tr;
		}
		if (final_position != end)
		{
			NVM_Transaction_Flash* tr = *end;
			queue->remove(end);
			queue->insert(final_position, tr);
		}
	}

	void TSU_SIMPLE_FLIN::estimate_alone_waiting_time(Flash_Transaction_Queue* queue, std::list<NVM_Transaction_Flash*>::iterator position)
	{
		sim_time_type chip_busy_time = 0, expected_last_time = 0;
		NVM_Transaction_Flash* chip_tr = _NVMController->Is_chip_busy_with_stream(*position);
		if (chip_tr && _NVMController->Expected_finish_time(chip_tr) > Simulator->Time())
		{
			chip_busy_time = _NVMController->Expected_finish_time(chip_tr) - Simulator->Time();
		}
		if (position != queue->begin())
		{
			auto itr = position;
			--itr;
			while (itr != queue->begin() && (*itr)->Stream_id != (*position)->Stream_id)
			{
				--itr;
			}
			if ((*itr)->Stream_id == (*position)->Stream_id)
			{
				expected_last_time = (*itr)->Estimated_alone_waiting_time
					+ _NVMController->Expected_transfer_time(*itr) + _NVMController->Expected_command_time(*itr);
				if (expected_last_time + (*itr)->Issue_time > Simulator->Time())
				{
					expected_last_time = expected_last_time + (*itr)->Issue_time - Simulator->Time();
				}
			}
		}
		(*position)->Estimated_alone_waiting_time = chip_busy_time + expected_last_time;
	}

	double TSU_SIMPLE_FLIN::fairness_based_on_average_slowdown(Flash_Transaction_Queue* queue, std::list<NVM_Transaction_Flash*>::iterator start,
		stream_id_type& flow_with_max_average_slowdown)
	{
		std::unordered_map<stream_id_type, double> sum_slowdown;
		std::unordered_map<stream_id_type, unsigned int> transaction_count;
		for (unsigned int stream_id = 0; stream_id < stream_count; stream_id++)
		{
			sum_slowdown[stream_id] = 0;
			transaction_count[stream_id] = 0;
		}
		sim_time_type total_finish_time = 0;
		auto itr = queue->begin();
		while (itr != start)
		{
			total_finish_time += _NVMController->Expected_transfer_time(*itr) + _NVMController->Expected_command_time(*itr);
			++itr;
		}
		while (itr != queue->end())
		{
			total_finish_time += _NVMController->Expected_transfer_time(*itr) + _NVMController->Expected_command_time(*itr);
			sim_time_type transaction_alone_time = (*itr)->Estimated_alone_waiting_time
				+ _NVMController->Expected_transfer_time(*itr) + _NVMController->Expected_command_time(*itr);
			sim_time_type transaction_shared_time = total_finish_time + (Simulator->Time() - (*itr)->Issue_time);
			sum_slowdown[(*itr)->Stream_id] += (double)transaction_shared_time / transaction_alone_time;
			++transaction_count[(*itr)->Stream_id];
			++itr;
		}

		double slowdown_max = 0, slowdown_min = DBL_MAX;
		unsigned int stream_count = 0;
		for (unsigned int stream_id = 0; stream_id < stream_count; stream_id++)
		{
			if (transaction_count[stream_id] == 0)
				continue;
			stream_count++;
			double average_slowdown = sum_slowdown[stream_id] / transaction_count[stream_id];
			if (average_slowdown > slowdown_max)
			{
				slowdown_max = average_slowdown;
				flow_with_max_average_slowdown = stream_id;
			}
			if (average_slowdown < slowdown_min)
				slowdown_min = average_slowdown;
		}
		if (stream_count == 1)
		{
			flow_with_max_average_slowdown = -1;
		}
		return (double)slowdown_min / (1e-10 + slowdown_max);
	}

	void TSU_SIMPLE_FLIN::move_forward(Flash_Transaction_Queue* queue, std::list<NVM_Transaction_Flash*>::iterator TRnew_pos, std::list<NVM_Transaction_Flash*>::iterator ultimate_posistion)
	{
		if (TRnew_pos == ultimate_posistion)
			return;
		auto Tnew_final_pos = TRnew_pos;
		--Tnew_final_pos;
		while ((*Tnew_final_pos)->Stream_id != (*TRnew_pos)->Stream_id && !(*Tnew_final_pos)->FLIN_Barrier && Tnew_final_pos != ultimate_posistion)
		{
			move_alone_time(*TRnew_pos, *Tnew_final_pos);
			--Tnew_final_pos;
		}

		if (Tnew_final_pos == ultimate_posistion
			&& (*Tnew_final_pos)->Stream_id != (*ultimate_posistion)->Stream_id
			&& !(*TRnew_pos)->FLIN_Barrier)
		{
			NVM_Transaction_Flash* tr = *TRnew_pos;
			queue->remove(TRnew_pos);
			queue->insert(Tnew_final_pos, tr);
			tr->FLIN_Barrier = true;//According to FLIN: When TRnew is moved forward, it is tagged so that no future arriving flash transaction of the high-intensity flows jumps ahead of it
		}
		else
		{
			NVM_Transaction_Flash* tr = *TRnew_pos;
			queue->remove(TRnew_pos);
			Tnew_final_pos--;
			queue->insert(Tnew_final_pos, tr);
			tr->FLIN_Barrier = true;//According to FLIN: When TRnew is moved forward, it is tagged so that no future arriving flash transaction of the high-intensity flows jumps ahead of it
		}
	}

	bool TSU_SIMPLE_FLIN::service_read_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		auto& info = transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].front();
		std::list<Flash_Transaction_Queue*> queue_list;
		if (!MappingReadTRQueue[chip->ChannelID][chip->ChipID].empty())
		{
			queue_list.push_back(&MappingReadTRQueue[chip->ChannelID][chip->ChipID]);
		}
		if (!UserReadTRQueue[chip->ChannelID][chip->ChipID].empty())
		{
			queue_list.push_back(&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
		}
		if (info.second == Transaction_Source_Type::USERIO)
		{
			if (queue_list.empty())
				return false;
			if (!GCReadTRQueue[chip->ChannelID][chip->ChipID].empty())
			{
				queue_list.push_back(&GCReadTRQueue[chip->ChannelID][chip->ChipID]);
			}
		}
		else if (info.second == Transaction_Source_Type::GC_WL)
		{
			if (!GCReadTRQueue[chip->ChannelID][chip->ChipID].empty())
			{
				queue_list.push_front(&GCReadTRQueue[chip->ChannelID][chip->ChipID]);
			}
			else return false;
		}
		bool suspensionRequired = false;
		ChipStatus cs = _NVMController->GetChipStatus(chip);
		switch (cs)
		{
		case ChipStatus::IDLE:
			break;
		case ChipStatus::WRITING:
			if (!programSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
				return false;
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < writeReasonableSuspensionTimeForRead)
				return false;
			suspensionRequired = true;
		case ChipStatus::ERASING:
			if (!eraseSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
				return false;
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < eraseReasonableSuspensionTimeForRead)
				return false;
			suspensionRequired = true;
		default:
			return false;
		}
		flash_die_ID_type die_id = queue_list.front()->front()->Address.DieID;
		for (unsigned int i = 0; i < die_no_per_chip; i++)
		{
			if (queue_list.empty())
			{
				break;
			}
			transaction_dispatch_slots.clear();
			unsigned int plane_vector = 0;
			flash_page_ID_type page_id = queue_list.front()->front()->Address.PageID;
			for (std::list<Flash_Transaction_Queue*>::iterator queue = queue_list.begin(); queue != queue_list.end(); )
			{
				for (Flash_Transaction_Queue::iterator it = (*queue)->begin(); it != (*queue)->end(); )
				{
					if ((*it)->Address.DieID == die_id && !(plane_vector & 1 << (*it)->Address.PlaneID))
					{
						if (plane_vector == 0)
						{
							page_id = (*it)->Address.PageID;
						}
						if ((*it)->Address.PageID == page_id)
						{
							if ((*it)->Source == Transaction_Source_Type::CACHE
								|| (*it)->Source == Transaction_Source_Type::USERIO)
							{
								if (it == head_high_read[chip->ChannelID][chip->ChipID])
								{
									++head_high_read[chip->ChannelID][chip->ChipID];
								}
							}
							stream_id_type dispatched_stream_id = (*it)->Stream_id;
							sim_time_type adjust_time = _NVMController->Expected_transfer_time(*it)
								+ _NVMController->Expected_command_time(*it);
							Transaction_Type type = (*it)->Type;
							Transaction_Source_Type source = (*it)->Source;
							(*it)->SuspendRequired = suspensionRequired;
							plane_vector |= 1 << (*it)->Address.PlaneID;
							transaction_dispatch_slots.push_back(*it);
							(*queue)->remove(it++);
							if (source == Transaction_Source_Type::MAPPING
								|| source == Transaction_Source_Type::GC_WL)
							{
								adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
									&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
							}
							adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
								&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
							continue;
						}
					}
					++it;
				}
				if ((*queue)->empty())
				{
					queue_list.erase(queue++);
					continue;
				}
				++queue;
				if (transaction_dispatch_slots.size() < plane_no_per_die)
				{
					break;
				}
			}
			if (!transaction_dispatch_slots.empty())
			{
				_NVMController->Send_command_to_chip(transaction_dispatch_slots);
			}
			die_id = (die_id + 1) % die_no_per_chip;
		}
		return true;
	}

	bool TSU_SIMPLE_FLIN::service_write_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		auto& info = transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].front();
		std::list<Flash_Transaction_Queue*> queue_list;
		/*if (!MappingWriteTRQueue[chip->ChannelID][chip->ChipID].empty())
		{
			queue_list.push_back(&MappingWriteTRQueue[chip->ChannelID][chip->ChipID]);
		}*/
		if (!UserWriteTRQueue[chip->ChannelID][chip->ChipID].empty())
		{
			queue_list.push_back(&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
		}
		if (info.second == Transaction_Source_Type::USERIO)
		{
			if (queue_list.empty())
				return false;
			if (!GCWriteTRQueue[chip->ChannelID][chip->ChipID].empty())
			{
				queue_list.push_back(&GCWriteTRQueue[chip->ChannelID][chip->ChipID]);
			}
		}
		else if (info.second == Transaction_Source_Type::GC_WL)
		{
			if (!GCWriteTRQueue[chip->ChannelID][chip->ChipID].empty())
			{
				queue_list.push_front(&GCWriteTRQueue[chip->ChannelID][chip->ChipID]);
			}
			else return false;
		}
		if (queue_list.empty())
			return false;
		bool suspensionRequired = false;
		ChipStatus cs = _NVMController->GetChipStatus(chip);
		switch (cs)
		{
		case ChipStatus::IDLE:
			break;
		case ChipStatus::ERASING:
			if (!eraseSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
				return false;
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < eraseReasonableSuspensionTimeForWrite)
				return false;
			suspensionRequired = true;
		default:
			return false;
		}
		flash_die_ID_type die_id = queue_list.front()->front()->Address.DieID;
		for (unsigned int i = 0; i < die_no_per_chip; i++)
		{
			if (queue_list.empty())
			{
				break;
			}
			transaction_dispatch_slots.clear();
			unsigned int plane_vector = 0;
			flash_page_ID_type page_id = queue_list.front()->front()->Address.PageID;
			for (std::list<Flash_Transaction_Queue*>::iterator queue = queue_list.begin(); queue != queue_list.end(); )
			{
				for (Flash_Transaction_Queue::iterator it = (*queue)->begin(); it != (*queue)->end(); )
				{
					if ((*it)->Address.DieID == die_id && !(plane_vector & 1 << (*it)->Address.PlaneID)
						&& ((NVM_Transaction_Flash_WR*)*it)->RelatedRead == NULL)
					{
						if (plane_vector == 0)
						{
							page_id = (*it)->Address.PageID;
						}
						if ((*it)->Address.PageID == page_id)
						{
							if ((*it)->Source == Transaction_Source_Type::CACHE
								|| (*it)->Source == Transaction_Source_Type::USERIO)
							{
								flow_activity_info[chip->ChannelID][chip->ChipID][(*it)->Stream_id].No_of_serviced_writes_since_last_GC++;
								if (it == head_high_write[chip->ChannelID][chip->ChipID])
								{
									++head_high_write[chip->ChannelID][chip->ChipID];
								}
							}
							stream_id_type dispatched_stream_id = (*it)->Stream_id;
							sim_time_type adjust_time = _NVMController->Expected_transfer_time(*it)
								+ _NVMController->Expected_command_time(*it);
							Transaction_Type type = (*it)->Type;
							Transaction_Source_Type source = (*it)->Source;
							(*it)->SuspendRequired = suspensionRequired;
							plane_vector |= 1 << (*it)->Address.PlaneID;
							transaction_dispatch_slots.push_back(*it);
							(*queue)->remove(it++);
							if (source == Transaction_Source_Type::MAPPING
								|| source == Transaction_Source_Type::GC_WL)
							{
								adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
									&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
							}
							adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
								&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
							continue;
						}
					}
					++it;
				}
				if ((*queue)->empty())
				{
					queue_list.erase(queue++);
					continue;
				}
				++queue;
				if (transaction_dispatch_slots.size() < plane_no_per_die)
				{
					break;
				}
			}
			if (!transaction_dispatch_slots.empty())
			{
				_NVMController->Send_command_to_chip(transaction_dispatch_slots);
			}
			die_id = (die_id + 1) % die_no_per_chip;
		}
		return true;
	}

	bool TSU_SIMPLE_FLIN::service_erase_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
			return false;

		Flash_Transaction_Queue* source_queue = &GCEraseTRQueue[chip->ChannelID][chip->ChipID];
		if (source_queue->size() == 0)
			return false;


		flash_die_ID_type die_id = source_queue->front()->Address.DieID;
		for (unsigned int i = 0; i < die_no_per_chip; i++)
		{
			if (source_queue->empty())
			{
				break;
			}
			transaction_dispatch_slots.clear();
			unsigned int plane_vector = 0;
			flash_block_ID_type block_id;
			block_id = source_queue->front()->Address.BlockID;

			for (Flash_Transaction_Queue::iterator it = source_queue->begin();
				it != source_queue->end() && transaction_dispatch_slots.size() < plane_no_per_die; )
			{
				if (((NVM_Transaction_Flash_ER*)*it)->Page_movement_activities.size() == 0 && (*it)->Address.DieID == die_id
					&& !(plane_vector & 1 << (*it)->Address.PlaneID))
				{
					if (plane_vector == 0)
					{
						block_id = (*it)->Address.BlockID;
					}
					if ((*it)->Address.BlockID == block_id)
					{
						stream_id_type dispatched_stream_id = (*it)->Stream_id;
						sim_time_type adjust_time = _NVMController->Expected_transfer_time(*it)
							+ _NVMController->Expected_command_time(*it);
						Transaction_Type type = (*it)->Type;
						plane_vector |= 1 << (*it)->Address.PlaneID;
						transaction_dispatch_slots.push_back(*it);
						source_queue->remove(it++);
						adjust_alone_time(dispatched_stream_id, adjust_time, type, Transaction_Source_Type::GC_WL,
							&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
						adjust_alone_time(dispatched_stream_id, adjust_time, type, Transaction_Source_Type::GC_WL,
							&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
						continue;
					}
				}
				++it;
			}
			if (!transaction_dispatch_slots.empty()) {
				_NVMController->Send_command_to_chip(transaction_dispatch_slots);
			}
			die_id = (die_id + 1) % die_no_per_chip;
		}
		for (unsigned int stream_id = 0; stream_id < stream_count; ++stream_id)
		{
			flow_activity_info[chip->ChannelID][chip->ChipID][stream_id].No_of_serviced_writes_since_last_GC = 0;
		}
		return true;
	}

	void TSU_SIMPLE_FLIN::service_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		/*if (!service_read_transaction0(chip))
			if (!service_write_transaction0(chip))
				service_erase_transaction0(chip);
		return;*/
		if (!MappingReadTRQueue[chip->ChannelID][chip->ChipID].empty())
		{
			transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].push_front(
				{ Transaction_Type::READ, Transaction_Source_Type::USERIO }
			);
		}
		// the number of valid pages for flow f in SSD cannot be calculated in MQSim
		unsigned int GCM = 0;
		NVM_Transaction_Flash_RD* read_slot = get_read_slot(chip->ChannelID, chip->ChipID);
		NVM_Transaction_Flash_WR* write_slot = get_write_slot(chip->ChannelID, chip->ChipID);
		if (!GCReadTRQueue[chip->ChannelID][chip->ChipID].empty())
		{
			unsigned int total_num_writes = 0;
			for (unsigned int stream_id = 0; stream_id < stream_count; ++stream_id)
			{
				total_num_writes += flow_activity_info[chip->ChannelID][chip->ChipID][stream_id].No_of_serviced_writes_since_last_GC;
			}
			if (total_num_writes != 0 && write_slot != NULL)
				GCM = (unsigned int)((double)GCReadTRQueue[chip->ChannelID][chip->ChipID].size()
					* (double)flow_activity_info[chip->ChannelID][chip->ChipID][write_slot->Stream_id].No_of_serviced_writes_since_last_GC
					/ total_num_writes);
		}
		double pw_read = 0, pw_write = 0;
		estimate_proportional_wait(read_slot, write_slot, pw_read, pw_write, GCM, chip->ChannelID, chip->ChipID);
		if (pw_read >= 0 && pw_read >= pw_write)
		{
			transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].push_back(
				{ Transaction_Type::READ, Transaction_Source_Type::USERIO });
		}
		else if (pw_write >= 0 && pw_write > pw_read)
		{

			if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				if (!service_read_transaction0(chip))
					if (!service_write_transaction0(chip))
						service_erase_transaction0(chip);
				service_write_transaction0(chip);
				return;
			}

			bool execute_gc = false;
			//if (ftl->BlockManager->Get_plane_bookkeeping_entry(write_slot->Address)->Free_pages_count < /*524288*//*GC_FLIN*/)
			{
				GCM = (int)GCReadTRQueue[chip->ChannelID][chip->ChipID].size();
				while (GCM > 0)
				{
					execute_gc = true;
					transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].push_back(
						{ Transaction_Type::READ, Transaction_Source_Type::GC_WL });
					transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].push_back(
						{ Transaction_Type::WRITE, Transaction_Source_Type::GC_WL });
					GCM--;
				}
			}
			transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].push_back(
				{ Transaction_Type::WRITE, Transaction_Source_Type::USERIO });
			if (execute_gc)
			{
				transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].push_back(
					{ Transaction_Type::ERASE, Transaction_Source_Type::GC_WL });
			}
		}
		bool success = false;
		while (!transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].empty() && !success)
		{
			switch (transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].front().first)
			{
			case Transaction_Type::READ:
				success = service_read_transaction(chip);
				break;
			case Transaction_Type::WRITE:
				success = service_write_transaction(chip);
				break;
			case Transaction_Type::ERASE:
				success = service_erase_transaction(chip);
				break;
			default:
				break;
			}
			transaction_waiting_dispatch_slots[chip->ChannelID][chip->ChipID].pop_front();
		}
		//if (Simulator->Time() >= 1.07e13)
		//{
		//	/*if (!service_read_transaction(chip))
		//		if (!service_write_transaction(chip))
		//			service_erase_transaction(chip);*/
		//	Simulator->Stop_simulation();
		//}
	}

	bool TSU_SIMPLE_FLIN::service_read_transaction0(NVM::FlashMemory::Flash_Chip* chip)
	{
		Flash_Transaction_Queue* sourceQueue1 = NULL, * sourceQueue2 = NULL;

		if (MappingReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)//Flash transactions that are related to FTL mapping data have the highest priority
		{
			sourceQueue1 = &MappingReadTRQueue[chip->ChannelID][chip->ChipID];
			if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip) && GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				sourceQueue2 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
			else if (UserReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				sourceQueue2 = &UserReadTRQueue[chip->ChannelID][chip->ChipID];
		}
		else if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip))//If flash transactions related to GC are prioritzed (non-preemptive execution mode of GC), then GC queues are checked first
		{
			if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue1 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
				if (UserReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
					sourceQueue2 = &UserReadTRQueue[chip->ChannelID][chip->ChipID];
			}
			else if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				return false;
			else if (GCEraseTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				return false;
			else if (UserReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				sourceQueue1 = &UserReadTRQueue[chip->ChannelID][chip->ChipID];
			else return false;
		}
		else //If GC is currently executed in the preemptive mode, then user IO transaction queues are checked first
		{
			if (UserReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue1 = &UserReadTRQueue[chip->ChannelID][chip->ChipID];
				if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
					sourceQueue2 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
			}
			else if (UserWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				return false;
			else if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				sourceQueue1 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
			else return false;
		}

		bool suspensionRequired = false;
		ChipStatus cs = _NVMController->GetChipStatus(chip);
		switch (cs)
		{
		case ChipStatus::IDLE:
			break;
		case ChipStatus::WRITING:
			if (!programSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
				return false;
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < writeReasonableSuspensionTimeForRead)
				return false;
			suspensionRequired = true;
		case ChipStatus::ERASING:
			if (!eraseSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
				return false;
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < eraseReasonableSuspensionTimeForRead)
				return false;
			suspensionRequired = true;
		default:
			return false;
		}

		flash_die_ID_type die_id = sourceQueue1->front()->Address.DieID;
		flash_page_ID_type page_id = sourceQueue1->front()->Address.PageID;
		unsigned int plane_vector = 0;
		for (unsigned int i = 0; i < die_no_per_chip; i++)
		{
			transaction_dispatch_slots.clear();
			plane_vector = 0;

			for (Flash_Transaction_Queue::iterator it = sourceQueue1->begin(); it != sourceQueue1->end();)
			{
				if ((*it)->Address.DieID == die_id && !(plane_vector & 1 << (*it)->Address.PlaneID))
				{
					if (plane_vector == 0)
					{
						page_id = (*it)->Address.PageID;
					}
					if ((*it)->Address.PageID == page_id)
					{
						if ((*it)->Source == Transaction_Source_Type::CACHE
							|| (*it)->Source == Transaction_Source_Type::USERIO)
						{
							if (it == head_high_read[chip->ChannelID][chip->ChipID])
							{
								++head_high_read[chip->ChannelID][chip->ChipID];
							}
						}
						stream_id_type dispatched_stream_id = (*it)->Stream_id;
						sim_time_type adjust_time = _NVMController->Expected_transfer_time(*it)
							+ _NVMController->Expected_command_time(*it);
						Transaction_Type type = (*it)->Type;
						Transaction_Source_Type source = (*it)->Source;
						(*it)->SuspendRequired = suspensionRequired;
						plane_vector |= 1 << (*it)->Address.PlaneID;
						transaction_dispatch_slots.push_back(*it);
						sourceQueue1->remove(it++);
						if (source == Transaction_Source_Type::MAPPING
							|| source == Transaction_Source_Type::GC_WL)
						{
							adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
								&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
						}
						adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
							&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
						continue;
					}
				}
				it++;
			}

			if (sourceQueue2 != NULL && transaction_dispatch_slots.size() < plane_no_per_die && transaction_dispatch_slots.empty())
				for (Flash_Transaction_Queue::iterator it = sourceQueue2->begin(); it != sourceQueue2->end();)
				{
					if ((*it)->Address.DieID == die_id && !(plane_vector & 1 << (*it)->Address.PlaneID))
					{
						if (plane_vector == 0)
						{
							page_id = (*it)->Address.PageID;
						}
						if ((*it)->Address.PageID == page_id)
						{
							if ((*it)->Source == Transaction_Source_Type::CACHE
								|| (*it)->Source == Transaction_Source_Type::USERIO)
							{
								if (it == head_high_read[chip->ChannelID][chip->ChipID])
								{
									++head_high_read[chip->ChannelID][chip->ChipID];
								}
							}
							stream_id_type dispatched_stream_id = (*it)->Stream_id;
							sim_time_type adjust_time = _NVMController->Expected_transfer_time(*it)
								+ _NVMController->Expected_command_time(*it);
							Transaction_Type type = (*it)->Type;
							Transaction_Source_Type source = (*it)->Source;
							(*it)->SuspendRequired = suspensionRequired;
							plane_vector |= 1 << (*it)->Address.PlaneID;
							transaction_dispatch_slots.push_back(*it);
							sourceQueue2->remove(it++);
							if (source == Transaction_Source_Type::MAPPING
								|| source == Transaction_Source_Type::GC_WL)
							{
								adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
									&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
							}
							adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
								&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
							continue;
						}
					}
					it++;
				}

			if (transaction_dispatch_slots.size() > 0)
				_NVMController->Send_command_to_chip(transaction_dispatch_slots);
			transaction_dispatch_slots.clear();
			die_id = (die_id + 1) % die_no_per_chip;
		}

		return true;
	}

	bool TSU_SIMPLE_FLIN::service_write_transaction0(NVM::FlashMemory::Flash_Chip* chip)
	{
		Flash_Transaction_Queue* sourceQueue1 = NULL, * sourceQueue2 = NULL;

		if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip))//If flash transactions related to GC are prioritzed (non-preemptive execution mode of GC), then GC queues are checked first
		{
			if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue1 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];
				if (UserWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
					sourceQueue2 = &UserWriteTRQueue[chip->ChannelID][chip->ChipID];
			}
			else if (GCEraseTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				return false;
			else if (UserWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				sourceQueue1 = &UserWriteTRQueue[chip->ChannelID][chip->ChipID];
			else return false;
		}
		else //If GC is currently executed in the preemptive mode, then user IO transaction queues are checked first
		{
			if (UserWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue1 = &UserWriteTRQueue[chip->ChannelID][chip->ChipID];
				if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
					sourceQueue2 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];
			}
			else if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				sourceQueue1 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];
			else return false;
		}


		bool suspensionRequired = false;
		ChipStatus cs = _NVMController->GetChipStatus(chip);
		switch (cs)
		{
		case ChipStatus::IDLE:
			break;
		case ChipStatus::ERASING:
			if (!eraseSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
				return false;
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < eraseReasonableSuspensionTimeForWrite)
				return false;
			suspensionRequired = true;
		default:
			return false;
		}

		flash_die_ID_type die_id = sourceQueue1->front()->Address.DieID;
		flash_page_ID_type page_id = sourceQueue1->front()->Address.PageID;
		unsigned int plane_vector = 0;
		for (unsigned int i = 0; i < die_no_per_chip; i++)
		{
			transaction_dispatch_slots.clear();
			plane_vector = 0;

			for (Flash_Transaction_Queue::iterator it = sourceQueue1->begin(); it != sourceQueue1->end(); )
			{
				if ((*it)->Address.DieID == die_id && !(plane_vector & 1 << (*it)->Address.PlaneID)
					&& ((NVM_Transaction_Flash_WR*)*it)->RelatedRead == NULL)
				{
					if (plane_vector == 0)
					{
						page_id = (*it)->Address.PageID;
					}
					if ((*it)->Address.PageID == page_id)
					{
						if ((*it)->Source == Transaction_Source_Type::CACHE
							|| (*it)->Source == Transaction_Source_Type::USERIO)
						{
							flow_activity_info[chip->ChannelID][chip->ChipID][(*it)->Stream_id].No_of_serviced_writes_since_last_GC++;
							if (it == head_high_write[chip->ChannelID][chip->ChipID])
							{
								++head_high_write[chip->ChannelID][chip->ChipID];
							}
						}
						stream_id_type dispatched_stream_id = (*it)->Stream_id;
						sim_time_type adjust_time = _NVMController->Expected_transfer_time(*it)
							+ _NVMController->Expected_command_time(*it);
						Transaction_Type type = (*it)->Type;
						Transaction_Source_Type source = (*it)->Source;
						(*it)->SuspendRequired = suspensionRequired;
						plane_vector |= 1 << (*it)->Address.PlaneID;
						transaction_dispatch_slots.push_back(*it);
						sourceQueue1->remove(it++);
						if (source == Transaction_Source_Type::MAPPING
							|| source == Transaction_Source_Type::GC_WL)
						{
							adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
								&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
						}
						adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
							&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
						continue;
					}
				}
			}

			if (sourceQueue2 != NULL && transaction_dispatch_slots.size() < plane_no_per_die && transaction_dispatch_slots.empty())
				for (Flash_Transaction_Queue::iterator it = sourceQueue2->begin(); it != sourceQueue2->end(); )
				{
					if ((*it)->Address.DieID == die_id && !(plane_vector & 1 << (*it)->Address.PlaneID)
						&& ((NVM_Transaction_Flash_WR*)*it)->RelatedRead == NULL)
					{
						if (plane_vector == 0)
						{
							page_id = (*it)->Address.PageID;
						}
						if ((*it)->Address.PageID == page_id)
						{
							if ((*it)->Source == Transaction_Source_Type::CACHE
								|| (*it)->Source == Transaction_Source_Type::USERIO)
							{
								flow_activity_info[chip->ChannelID][chip->ChipID][(*it)->Stream_id].No_of_serviced_writes_since_last_GC++;
								if (it == head_high_write[chip->ChannelID][chip->ChipID])
								{
									++head_high_write[chip->ChannelID][chip->ChipID];
								}
							}
							stream_id_type dispatched_stream_id = (*it)->Stream_id;
							sim_time_type adjust_time = _NVMController->Expected_transfer_time(*it)
								+ _NVMController->Expected_command_time(*it);
							Transaction_Type type = (*it)->Type;
							Transaction_Source_Type source = (*it)->Source;
							(*it)->SuspendRequired = suspensionRequired;
							plane_vector |= 1 << (*it)->Address.PlaneID;
							transaction_dispatch_slots.push_back(*it);
							sourceQueue2->remove(it++);
							if (source == Transaction_Source_Type::MAPPING
								|| source == Transaction_Source_Type::GC_WL)
							{
								adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
									&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
							}
							adjust_alone_time(dispatched_stream_id, adjust_time, type, source,
								&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
							continue;
						}
					}
				}

			if (transaction_dispatch_slots.size() > 0)
				_NVMController->Send_command_to_chip(transaction_dispatch_slots);
			transaction_dispatch_slots.clear();
			die_id = (die_id + 1) % die_no_per_chip;
		}
		return true;
	}

	bool TSU_SIMPLE_FLIN::service_erase_transaction0(NVM::FlashMemory::Flash_Chip* chip)
	{
		if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
			return false;

		Flash_Transaction_Queue* source_queue = &GCEraseTRQueue[chip->ChannelID][chip->ChipID];
		if (source_queue->size() == 0)
			return false;

		flash_die_ID_type die_id = source_queue->front()->Address.DieID;
		flash_block_ID_type block_id = source_queue->front()->Address.DieID;
		unsigned int plane_vector = 0;
		for (unsigned int i = 0; i < die_no_per_chip; i++)
		{
			transaction_dispatch_slots.clear();
			plane_vector = 0;

			for (Flash_Transaction_Queue::iterator it = source_queue->begin(); it != source_queue->end(); )
			{
				if (((NVM_Transaction_Flash_ER*)*it)->Page_movement_activities.size() == 0 && (*it)->Address.DieID == die_id
					&& !(plane_vector & 1 << (*it)->Address.PlaneID))
				{
					if (plane_vector == 0)
					{
						block_id = (*it)->Address.BlockID;
					}
					if ((*it)->Address.BlockID == block_id)
					{
						stream_id_type dispatched_stream_id = (*it)->Stream_id;
						sim_time_type adjust_time = _NVMController->Expected_transfer_time(*it)
							+ _NVMController->Expected_command_time(*it);
						Transaction_Type type = (*it)->Type;
						plane_vector |= 1 << (*it)->Address.PlaneID;
						transaction_dispatch_slots.push_back(*it);
						source_queue->remove(it++);
						adjust_alone_time(dispatched_stream_id, adjust_time, type, Transaction_Source_Type::GC_WL,
							&UserReadTRQueue[chip->ChannelID][chip->ChipID]);
						adjust_alone_time(dispatched_stream_id, adjust_time, type, Transaction_Source_Type::GC_WL,
							&UserWriteTRQueue[chip->ChannelID][chip->ChipID]);
						continue;
					}
				}
				it++;
			}
			if (transaction_dispatch_slots.size() > 0)
				_NVMController->Send_command_to_chip(transaction_dispatch_slots);
			transaction_dispatch_slots.clear();
			die_id = (die_id + 1) % die_no_per_chip;
		}
		for (unsigned int stream_id = 0; stream_id < stream_count; ++stream_id)
		{
			flow_activity_info[chip->ChannelID][chip->ChipID][stream_id].No_of_serviced_writes_since_last_GC = 0;
		}
		return true;
	}

	void TSU_SIMPLE_FLIN::estimate_alone_time(NVM_Transaction_Flash* transaction, Flash_Transaction_Queue* queue)
	{
		sim_time_type chip_busy_time = 0, read_waiting_last_time = 0, write_waiting_last_time = 0;
		NVM_Transaction_Flash* chip_tr = _NVMController->Is_chip_busy_with_stream(transaction);
		if (chip_tr && _NVMController->Expected_finish_time(chip_tr) > Simulator->Time())
		{
			chip_busy_time = _NVMController->Expected_finish_time(chip_tr) - Simulator->Time();
		}
		if (queue->size())
		{
			auto itr_it = queue->end();
			do
			{
				--itr_it;
				if ((*itr_it)->Stream_id == transaction->Stream_id)
				{
					if ((*itr_it)->Type == Transaction_Type::READ)
						read_waiting_last_time += _NVMController->Expected_command_time(*itr_it) + _NVMController->Expected_transfer_time(*itr_it);
					else if ((*itr_it)->Type == Transaction_Type::WRITE)
						write_waiting_last_time += _NVMController->Expected_command_time(*itr_it) + _NVMController->Expected_transfer_time(*itr_it);
				}
			} while (itr_it != queue->begin());
		}
		read_waiting_last_time += read_waiting_last_time / 2;
		write_waiting_last_time /= 2;
		transaction->alone_time = chip_busy_time + read_waiting_last_time + write_waiting_last_time
			+ _NVMController->Expected_transfer_time(transaction) + _NVMController->Expected_command_time(transaction);
	}

	void TSU_SIMPLE_FLIN::move_alone_time(NVM_Transaction_Flash* forward_transaction, NVM_Transaction_Flash* backward_transaction)
	{
		sim_time_type forward_time = _NVMController->Expected_command_time(forward_transaction) + _NVMController->Expected_transfer_time(forward_transaction);
		sim_time_type backward_time = _NVMController->Expected_command_time(backward_transaction) + _NVMController->Expected_transfer_time(backward_transaction);
		if (forward_transaction->Type == Transaction_Type::READ)
		{
			forward_time += forward_time / 2;
			backward_time += backward_time / 2;
		}
		else if (forward_transaction->Type == Transaction_Type::WRITE)
		{
			forward_time /= 2;
			backward_time /= 2;
		}
		forward_transaction->alone_time -= forward_time;
		backward_transaction->alone_time += forward_time;
	}

	void TSU_SIMPLE_FLIN::adjust_alone_time(stream_id_type dispatched_stream_id, sim_time_type adjust_time, Transaction_Type type, Transaction_Source_Type source, Flash_Transaction_Queue* queue)
	{
		if (source == Transaction_Source_Type::CACHE || source == Transaction_Source_Type::USERIO)
		{
			if (type == Transaction_Type::READ)
			{
				adjust_time += adjust_time / 2;
			}
			else if (type == Transaction_Type::WRITE)
			{
				adjust_time /= 4;
			}
		}
		else if (source == Transaction_Source_Type::GC_WL)
		{
			if (type == Transaction_Type::READ)
			{
				adjust_time += adjust_time / 2/*+= adjust_time / 2*/;
			}
			else if (type == Transaction_Type::WRITE)
			{
				adjust_time -= adjust_time / 3/*+= adjust_time / 3*/;
			}
			else if (type == Transaction_Type::ERASE)
			{
				adjust_time/*+= adjust_time / 2*/;
			}
		}
		else if (source == Transaction_Source_Type::MAPPING)
		{
		}
		for (auto it = queue->begin(); it != queue->end(); ++it)
		{
			if (dispatched_stream_id == (*it)->Stream_id)
			{
				(*it)->alone_time += adjust_time;
			}
		}
	}

	void TSU_SIMPLE_FLIN::estimate_proportional_wait(NVM_Transaction_Flash_RD* read_slot, NVM_Transaction_Flash_WR* write_slot,
		double& pw_read, double& pw_write, unsigned int GCM, flash_channel_ID_type channel_id, flash_chip_ID_type chip_id)
	{
		sim_time_type read_cost = 0;
		sim_time_type write_cost = 0;
		sim_time_type T_erase_memory = 0;
		pw_read = -1;
		pw_write = -1;
		if (read_slot)
		{
			read_cost = _NVMController->Expected_transfer_time(read_slot) + _NVMController->Expected_finish_time(read_slot);
		}
		if (write_slot)
		{
			write_cost = _NVMController->Expected_transfer_time(write_slot) + _NVMController->Expected_finish_time(write_slot);
		}
		if (!GCEraseTRQueue[channel_id][chip_id].empty())
		{
			T_erase_memory = _NVMController->Expected_finish_time(GCEraseTRQueue[channel_id][chip_id].front());
		}
		sim_time_type T_GC = GCM == 0 ? 0 : GCM * (read_cost + write_cost) + T_erase_memory;
		if (read_cost != 0)
		{
			pw_read = (double)(Simulator->Time() - read_slot->Issue_time + write_cost + T_GC) / read_cost;
		}
		if (write_cost != 0)
		{
			pw_write = (double)(Simulator->Time() - write_slot->Issue_time + read_cost + T_GC) / write_cost;
		}
	}

	NVM_Transaction_Flash_RD* TSU_SIMPLE_FLIN::get_read_slot(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id)
	{
		NVM_Transaction_Flash* slot = NULL;
		if (MappingReadTRQueue[channel_id][chip_id].empty())
		{
			if (!UserReadTRQueue[channel_id][chip_id].empty())
			{
				slot = UserReadTRQueue[channel_id][chip_id].front();
			}
		}
		else
		{
			slot = MappingReadTRQueue[channel_id][chip_id].front();
		}
		return (NVM_Transaction_Flash_RD*)slot;
	}

	NVM_Transaction_Flash_WR* TSU_SIMPLE_FLIN::get_write_slot(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id)
	{
		NVM_Transaction_Flash* slot = NULL;
		if (MappingWriteTRQueue[channel_id][chip_id].empty())
		{
			slot = UserWriteTRQueue[channel_id][chip_id].empty() ? NULL : UserWriteTRQueue[channel_id][chip_id].front();
		}
		/*else
		{
			slot = MappingWriteTRQueue[channel_id][chip_id].front();
		}*/
		return (NVM_Transaction_Flash_WR*)slot;
	}

	void TSU_SIMPLE_FLIN::Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter)
	{
		name_prefix = name_prefix + +".TSU";
		xmlwriter.Write_open_tag(name_prefix);

		TSU_Base::Report_results_in_XML(name_prefix, xmlwriter);

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
				UserReadTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".User_Read_TR_Queue", xmlwriter);

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
				UserWriteTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".User_Write_TR_Queue", xmlwriter);

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
				MappingReadTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".Mapping_Read_TR_Queue", xmlwriter);

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
				MappingWriteTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".Mapping_Write_TR_Queue", xmlwriter);

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
				GCReadTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Read_TR_Queue", xmlwriter);

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
				GCWriteTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Write_TR_Queue", xmlwriter);

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
				GCEraseTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Erase_TR_Queue", xmlwriter);

		xmlwriter.Write_close_tag();
	}

}