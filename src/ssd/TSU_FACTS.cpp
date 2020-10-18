#include "TSU_FACTS.h"

namespace SSD_Components
{
	TSU_FACTS::TSU_FACTS(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI_NVDDR2* NVMController,
		unsigned int channel_count, unsigned int chip_count_per_channel, unsigned int die_count_per_chip,
		unsigned int plane_count_per_die, unsigned int StreamCount, sim_time_type WriteReasonableSuspensionTimeForRead,
		sim_time_type EraseReasonableSuspensionTimeForRead, sim_time_type EraseReasonableSuspensionTimeForWrite,
		bool EraseSuspensionEnabled, bool ProgramSuspensionEnabled)
		: TSU_Base(id, ftl, NVMController, Flash_Scheduling_Type::FACTS, channel_count, chip_count_per_channel,
			die_count_per_chip, plane_count_per_die, WriteReasonableSuspensionTimeForRead, EraseReasonableSuspensionTimeForRead,
			EraseReasonableSuspensionTimeForWrite, EraseSuspensionEnabled, ProgramSuspensionEnabled),
		stream_count(StreamCount)
	{
		UserTRQueue = new Flash_Transaction_Queue * [channel_count];
		GCReadTRQueue = new Flash_Transaction_Queue * [channel_count];
		GCWriteTRQueue = new Flash_Transaction_Queue * [channel_count];
		GCEraseTRQueue = new Flash_Transaction_Queue * [channel_count];
		MappingReadTRQueue = new Flash_Transaction_Queue * [channel_count];
		MappingWriteTRQueue = new Flash_Transaction_Queue * [channel_count];

		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
		{
			UserTRQueue[channel_id] = new Flash_Transaction_Queue[chip_count_per_channel];
			GCReadTRQueue[channel_id] = new Flash_Transaction_Queue[chip_count_per_channel];
			GCWriteTRQueue[channel_id] = new Flash_Transaction_Queue[chip_count_per_channel];
			GCEraseTRQueue[channel_id] = new Flash_Transaction_Queue[chip_count_per_channel];
			MappingReadTRQueue[channel_id] = new Flash_Transaction_Queue[chip_count_per_channel];
			MappingWriteTRQueue[channel_id] = new Flash_Transaction_Queue[chip_count_per_channel];

			for (unsigned int chip_id = 0; chip_id < chip_count_per_channel; chip_id++)
			{
				std::string str = std::to_string(channel_id) + "@" + std::to_string(chip_id);
				UserTRQueue[channel_id][chip_id].Set_id("User_Read_TR_Queue@" + str);
				GCReadTRQueue[channel_id][chip_id].Set_id("GC_Read_TR_Queue@" + str);
				MappingReadTRQueue[channel_id][chip_id].Set_id("Mapping_Read_TR_Queue@" + str);
				MappingWriteTRQueue[channel_id][chip_id].Set_id("Mapping_Write_TR_Queue@" + str);
				GCWriteTRQueue[channel_id][chip_id].Set_id("GC_Write_TR_Queue@" + str);
				GCEraseTRQueue[channel_id][chip_id].Set_id("GC_Erase_TR_Queue@" + str);
			}
		}

		buffer_total_time = new sim_time_type[stream_count];
		queue_total_time = new sim_time_type[stream_count];
		backend_total_time = new sim_time_type[stream_count];
		shared_total_time = new sim_time_type[stream_count];
		alone_total_time = new sim_time_type[stream_count];
		shared_read_total_time = new sim_time_type[stream_count];
		alone_read_total_time = new sim_time_type[stream_count];
		shared_write_total_time = new sim_time_type[stream_count];
		alone_write_total_time = new sim_time_type[stream_count];
		total_count = new unsigned long long[stream_count];
		read_total_count = new unsigned long long[stream_count];
		write_total_count = new unsigned long long[stream_count];
		slowdown = new double[stream_count];

		for (unsigned int stream_id = 0; stream_id < stream_count; ++stream_id)
		{
			buffer_total_time[stream_id] = 0;
			queue_total_time[stream_id] = 0;
			backend_total_time[stream_id] = 0;
			shared_total_time[stream_id] = 0;
			alone_total_time[stream_id] = 0;
			total_count[stream_id] = 0;
			shared_read_total_time[stream_id] = 0;
			alone_read_total_time[stream_id] = 0;
			read_total_count[stream_id] = 0;
			shared_write_total_time[stream_id] = 0;
			alone_write_total_time[stream_id] = 0;
			write_total_count[stream_id] = 0;
			slowdown[stream_id] = 0;
		}
	}

	TSU_FACTS::~TSU_FACTS()
	{

		for (stream_id_type stream_id = 0; stream_id < stream_count; ++stream_id)
		{
			std::cout << "\n==========\n";
			std::cout << "total"
				<< "\tshared\t" << shared_total_time[stream_id] / total_count[stream_id] / 1000
				<< "\talone\t" << alone_total_time[stream_id] / total_count[stream_id] / 1000
				<< "\tslowdown\t" << (double)shared_total_time[stream_id] / alone_total_time[stream_id]
				<< "\taverage slowdown\t" << slowdown[stream_id] / total_count[stream_id]
				<< "\nread"
				<< "\tshared\t" << shared_read_total_time[stream_id] / (1e-5 + read_total_count[stream_id]) / 1000
				<< "\talone\t" << alone_read_total_time[stream_id] / (1e-5 + read_total_count[stream_id]) / 1000
				<< "\tslowdown\t" << (double)shared_read_total_time[stream_id] / (alone_read_total_time[stream_id] + 1e-5)
				<< "\nwrite"
				<< "\tshared\t" << shared_write_total_time[stream_id] / (1e-5 + write_total_count[stream_id]) / 1000
				<< "\talone\t" << alone_write_total_time[stream_id] / (1e-5 + write_total_count[stream_id]) / 1000
				<< "\tslowdown\t" << (double)shared_write_total_time[stream_id] / (alone_write_total_time[stream_id] + 1e-5)
				<< "\nbuffer\t" << buffer_total_time[stream_id] / total_count[stream_id] / 1000
				<< "\tqueue\t" << queue_total_time[stream_id] / total_count[stream_id] / 1000
				<< "\tbackend\t" << backend_total_time[stream_id] / total_count[stream_id] / 1000;
		}
		std::cout << "\n==========\n";

		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
		{
			delete[] UserTRQueue[channel_id];
			delete[] GCReadTRQueue[channel_id];
			delete[] GCWriteTRQueue[channel_id];
			delete[] GCEraseTRQueue[channel_id];
			delete[] MappingReadTRQueue[channel_id];
			delete[] MappingWriteTRQueue[channel_id];
		}
		delete[] UserTRQueue;
		delete[] GCReadTRQueue;
		delete[] GCWriteTRQueue;
		delete[] GCEraseTRQueue;
		delete[] MappingReadTRQueue;
		delete[] MappingWriteTRQueue;

		delete[] buffer_total_time;
		delete[] queue_total_time;
		delete[] backend_total_time;
		delete[] shared_total_time;
		delete[] alone_total_time;
		delete[] total_count;
		delete[] shared_read_total_time;
		delete[] alone_read_total_time;
		delete[] read_total_count;
		delete[] shared_write_total_time;
		delete[] alone_write_total_time;
		delete[] write_total_count;
		delete[] slowdown;
	}

	void TSU_FACTS::Start_simulation()
	{
		if (stream_count > 1)
		{
			Simulator->Register_sim_event(1000000, this, 0, (int)Transaction_Type::READ);
			Simulator->Register_sim_event(2000000, this, 0, (int)Transaction_Type::WRITE);
		}
	}
	void TSU_FACTS::Validate_simulation_config() {}

	void TSU_FACTS::Execute_simulator_event(MQSimEngine::Sim_Event* event)
	{
	}

	void TSU_FACTS::handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash* transaction)
	{
		if (transaction->Source == Transaction_Source_Type::GC_WL || transaction->Source == Transaction_Source_Type::MAPPING)
			return;
		total_count[transaction->Stream_id] += 1;
		sim_time_type buffer_time = transaction->queue_time - transaction->buffer_time;
		sim_time_type queue_time = transaction->backend_time - transaction->queue_time;
		sim_time_type backend_time = Simulator->Time() - transaction->backend_time;
		buffer_total_time[transaction->Stream_id] += buffer_time;
		queue_total_time[transaction->Stream_id] += queue_time;
		backend_total_time[transaction->Stream_id] += backend_time;
		alone_total_time[transaction->Stream_id] += transaction->alone_time;
		//shared_total_time[transaction->Stream_id] += transaction->shared_time;
		//slowdown[transaction->Stream_id] += (double)transaction->shared_time / transaction->alone_time;
		shared_total_time[transaction->Stream_id] += Simulator->Time() - transaction->Issue_time;
		slowdown[transaction->Stream_id] += (double)(Simulator->Time() - transaction->Issue_time) / transaction->alone_time;
		if (transaction->Type == Transaction_Type::READ)
		{
			shared_read_total_time[transaction->Stream_id] += Simulator->Time() - transaction->Issue_time;
			//shared_read_total_time[transaction->Stream_id] += transaction->shared_time;
			alone_read_total_time[transaction->Stream_id] += transaction->alone_time;
			read_total_count[transaction->Stream_id] += 1;
		}
		else if (transaction->Type == Transaction_Type::WRITE)
		{
			shared_write_total_time[transaction->Stream_id] += Simulator->Time() - transaction->Issue_time;
			//shared_write_total_time[transaction->Stream_id] += transaction->shared_time;
			alone_write_total_time[transaction->Stream_id] += transaction->alone_time;
			write_total_count[transaction->Stream_id] += 1;
		}
		sim_time_type st = Simulator->Time() - transaction->Issue_time;
	}

	inline void TSU_FACTS::Prepare_for_transaction_submit()
	{
		opened_scheduling_reqs++;
		if (opened_scheduling_reqs > 1)
			return;
		transaction_receive_slots.clear();
	}

	inline void TSU_FACTS::Submit_transaction(NVM_Transaction_Flash* transaction)
	{
		transaction_receive_slots.push_back(transaction);
	}

	void TSU_FACTS::Schedule()
	{
		opened_scheduling_reqs--;
		if (opened_scheduling_reqs > 0)
			return;
		if (opened_scheduling_reqs < 0)
			PRINT_ERROR("TSU_FACTS: Illegal status!");

		if (transaction_receive_slots.size() == 0)
			return;

		for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_receive_slots.begin();
			it != transaction_receive_slots.end(); it++)
		{
			switch ((*it)->Type)
			{
			case Transaction_Type::READ:
				switch ((*it)->Source)
				{
				case Transaction_Source_Type::CACHE:
				case Transaction_Source_Type::USERIO:
					estimate_time(*it, &UserTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID]);
					(*it)->buffer_time = Simulator->Time();
					UserTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
					(*it)->queue_time = Simulator->Time();
					break;
				case Transaction_Source_Type::MAPPING:
					MappingReadTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
					break;
				case Transaction_Source_Type::GC_WL:
					GCReadTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
					break;
				default:
					PRINT_ERROR("TSU_FACTS: unknown source type for a read transaction!")
				}
				break;
			case Transaction_Type::WRITE:
				switch ((*it)->Source)
				{
				case Transaction_Source_Type::CACHE:
				case Transaction_Source_Type::USERIO:
					estimate_time(*it, &UserTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID]);
					(*it)->buffer_time = Simulator->Time();
					UserTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
					(*it)->queue_time = Simulator->Time();
					break;
				case Transaction_Source_Type::MAPPING:
					MappingWriteTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
					break;
				case Transaction_Source_Type::GC_WL:
					GCWriteTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
					break;
				default:
					PRINT_ERROR("TSU_FACTS: unknown source type for a write transaction!")
				}
				break;
			case Transaction_Type::ERASE:
				GCEraseTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
				break;
			default:
				break;
			}
		}
		for (flash_channel_ID_type channel_id = 0; channel_id < channel_count; channel_id++)
		{
			if (_NVMController->Get_channel_status(channel_id) == BusChannelStatus::IDLE)
			{
				for (unsigned int i = 0; i < chip_no_per_channel; i++)
				{
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

	void TSU_FACTS::estimate_time(NVM_Transaction_Flash* transaction, Flash_Transaction_Queue* user_queue)
	{
		sim_time_type chip_busy_time = 0;
		sim_time_type shared_waiting_read_time = 0, shared_waiting_write_time = 0;
		sim_time_type alone_waiting_read_time = 0, alone_waiting_write_time = 0;
		NVM_Transaction_Flash* chip_tr = _NVMController->Is_chip_busy_with_stream(transaction);
		if (chip_tr && _NVMController->Expected_finish_time(chip_tr) > Simulator->Time())
		{
			chip_busy_time = _NVMController->Expected_finish_time(chip_tr) - Simulator->Time();
		}
		if (user_queue->size())
		{
			auto it = user_queue->end();
			do
			{
				--it;
				sim_time_type transaction_time = _NVMController->Expected_transfer_time(*it) + _NVMController->Expected_command_time(*it);
				switch ((*it)->Type)
				{
				case Transaction_Type::READ:
					if ((*it)->Stream_id == transaction->Stream_id)
					{
						alone_waiting_read_time += transaction_time;
					}
					shared_waiting_read_time += transaction_time;
					break;
				case Transaction_Type::WRITE:
					if ((*it)->Stream_id == transaction->Stream_id)
					{
						alone_waiting_write_time += transaction_time;
					}
					shared_waiting_write_time += transaction_time;
					break;
				default:
					break;
				}
			} while (it != user_queue->begin());
			alone_waiting_read_time += alone_waiting_read_time / 2;
			shared_waiting_read_time += shared_waiting_read_time / 2;
			alone_waiting_write_time /= 4;
			shared_waiting_write_time /= 4;
		}
		transaction->alone_time = chip_busy_time + alone_waiting_read_time + alone_waiting_write_time
			+ _NVMController->Expected_transfer_time(transaction) + _NVMController->Expected_command_time(transaction);
		transaction->shared_time = chip_busy_time + shared_waiting_read_time + shared_waiting_write_time
			+ _NVMController->Expected_transfer_time(transaction) + _NVMController->Expected_command_time(transaction);
	}

	bool TSU_FACTS::service_read_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		bool suspensionRequired = false;
		std::list<Flash_Transaction_Queue*> queue_list;
		if (MappingReadTRQueue[chip->ChannelID][chip->ChipID].size())
		{
			queue_list.push_back(&MappingReadTRQueue[chip->ChannelID][chip->ChipID]);
		}
		if (UserTRQueue[chip->ChannelID][chip->ChipID].size())
		{
			queue_list.push_back(&UserTRQueue[chip->ChannelID][chip->ChipID]);
		}
		if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size())
		{
			if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip))
			{
				queue_list.push_front(&GCReadTRQueue[chip->ChannelID][chip->ChipID]);
			}
			else
			{
				queue_list.push_back(&GCReadTRQueue[chip->ChannelID][chip->ChipID]);
			}
		}
		if (queue_list.empty())
			return false;
		flash_die_ID_type die_id = queue_list.front()->front()->Address.DieID;
		for (unsigned int i = 0; i < die_no_per_chip; i++)
		{
			if (queue_list.empty())
			{
				break;
			}
			transaction_dispatch_slots.clear();
			unsigned int plane_vector = 0;
			flash_page_ID_type page_id = -1;
			for (std::list<Flash_Transaction_Queue*>::iterator queue = queue_list.begin(); queue != queue_list.end(); )
			{
				for (Flash_Transaction_Queue::iterator it = (*queue)->begin(); it != (*queue)->end(); )
				{
					if ((*it)->Address.DieID == die_id && !(plane_vector & 1 << (*it)->Address.PlaneID)
						&& (*it)->Type == Transaction_Type::READ)
					{
						if (plane_vector == 0)
						{
							page_id = (*it)->Address.PageID;
						}
						if ((*it)->Address.PageID == page_id)
						{
							if ((*it)->Source == Transaction_Source_Type::CACHE
								|| (*it)->Source == Transaction_Source_Type::USERIO
								|| (*it)->Source == Transaction_Source_Type::MAPPING)
							{
								(*it)->backend_time = Simulator->Time();
							}
							(*it)->SuspendRequired = suspensionRequired;
							plane_vector |= 1 << (*it)->Address.PlaneID;
							transaction_dispatch_slots.push_back(*it);
							(*queue)->remove(it++);
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

	bool TSU_FACTS::service_write_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		bool suspensionRequired = false;
		std::list<Flash_Transaction_Queue*> queue_list;
		/*if (MappingWriteTRQueue[chip->ChannelID][chip->ChipID].size())
		{
			queue_list.push_back(&MappingWriteTRQueue[chip->ChannelID][chip->ChipID]);
		}*/
		if (UserTRQueue[chip->ChannelID][chip->ChipID].size())
		{
			queue_list.push_back(&UserTRQueue[chip->ChannelID][chip->ChipID]);
		}
		if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size())
		{
			if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip))
			{
				queue_list.push_front(&GCWriteTRQueue[chip->ChannelID][chip->ChipID]);
			}
			else
			{
				queue_list.push_back(&GCWriteTRQueue[chip->ChannelID][chip->ChipID]);
			}
		}
		if (queue_list.empty())
			return false;
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
						&& ((NVM_Transaction_Flash_WR*)*it)->RelatedRead == NULL
						&& (*it)->Type == Transaction_Type::WRITE)
					{
						if (plane_vector == 0)
						{
							page_id = (*it)->Address.PageID;
						}
						if ((*it)->Address.PageID == page_id)
						{
							if ((*it)->Source == Transaction_Source_Type::CACHE
								|| (*it)->Source == Transaction_Source_Type::USERIO
								/*|| (*it)->Source == Transaction_Source_Type::MAPPING*/)
							{
								(*it)->backend_time = Simulator->Time();
							}
							(*it)->SuspendRequired = suspensionRequired;
							plane_vector |= 1 << (*it)->Address.PlaneID;
							transaction_dispatch_slots.push_back(*it);
							(*queue)->remove(it++);
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

	bool TSU_FACTS::service_erase_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
			return false;

		Flash_Transaction_Queue* source_queue = &GCEraseTRQueue[chip->ChannelID][chip->ChipID];
		if (source_queue->size() == 0)
			return false;

		flash_die_ID_type die_id = source_queue->front()->Address.DieID;
		for (unsigned int i = 0; i < die_no_per_chip; i++) {
			if (source_queue->empty())
			{
				break;
			}
			transaction_dispatch_slots.clear();
			unsigned int plane_vector = 0;
			flash_block_ID_type block_id = source_queue->front()->Address.BlockID;

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
						plane_vector |= 1 << (*it)->Address.PlaneID;
						transaction_dispatch_slots.push_back(*it);
						source_queue->remove(it++);
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
		return true;
	}

	void TSU_FACTS::service_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
			return;
		if (UserTRQueue[chip->ChannelID][chip->ChipID].empty())
		{
			if (!service_read_transaction(chip))
				if (!service_write_transaction(chip))
					service_erase_transaction(chip);
			return;
		}
		bool success = false;
		switch (UserTRQueue[chip->ChannelID][chip->ChipID].front()->Type)
		{
		case Transaction_Type::READ:
			success = service_read_transaction(chip);
			break;
		case Transaction_Type::WRITE:
			success = service_write_transaction(chip);
			break;
		default:
			break;
		}
		if (!success)
		{
			service_erase_transaction(chip);
		}
	}

	void TSU_FACTS::Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter)
	{
		name_prefix = name_prefix + +".TSU";
		xmlwriter.Write_open_tag(name_prefix);

		TSU_Base::Report_results_in_XML(name_prefix, xmlwriter);

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
				UserTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".User_TR_Queue", xmlwriter);

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