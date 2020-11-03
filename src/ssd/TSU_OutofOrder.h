#ifndef TSU_OUTOFORDER_H
#define TSU_OUTOFORDER_H

#include <list>
#include "TSU_Base.h"
#include "NVM_Transaction_Flash.h"
#include "NVM_PHY_ONFI_NVDDR2.h"
#include "FTL.h"

namespace SSD_Components
{
	class FTL;

	/*
	* This class implements a transaction scheduling unit which supports:
	* 1. Out-of-order execution of flash transactions, similar to the Sprinkler proposal
	*    described in "Jung et al., Sprinkler: Maximizing resource utilization in many-chip
	*    solid state disks, HPCA, 2014".
	* 2. Program and erase suspension, similar to the proposal described in "G. Wu and X. He,
	*    Reducing SSD read latency via NAND flash program and erase suspension, FAST 2012".
	*/
	class TSU_OutOfOrder : public TSU_Base
	{
	public:
		TSU_OutOfOrder(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI_NVDDR2* NVMController, unsigned int Channel_no, unsigned int chip_no_per_channel,
			unsigned int DieNoPerChip, unsigned int PlaneNoPerDie, unsigned int StreamCount,
			sim_time_type WriteReasonableSuspensionTimeForRead,
			sim_time_type EraseReasonableSuspensionTimeForRead,
			sim_time_type EraseReasonableSuspensionTimeForWrite,
			bool EraseSuspensionEnabled, bool ProgramSuspensionEnabled);
		~TSU_OutOfOrder();
		void Prepare_for_transaction_submit();
		void Submit_transaction(NVM_Transaction_Flash* transaction);
		void handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash* transaction);
		void Schedule();

		void Start_simulation();
		void Validate_simulation_config();
		void Execute_simulator_event(MQSimEngine::Sim_Event*);
		void Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter);

		double proportional_slowdown(stream_id_type gc_stream_id);
		size_t GCEraseTRQueueSize(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id)
		{
			return GCEraseTRQueue[channel_id][chip_id].size();
		}
		double fairness();
		double proportional_slowdown(stream_id_type gc_stream_id, flash_channel_ID_type channel_id, flash_chip_ID_type chip_id)
		{
			return proportional_slowdown(gc_stream_id);

		}
		double fairness(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id) { return fairness(); }

	private:
		Flash_Transaction_Queue** UserReadTRQueue;
		Flash_Transaction_Queue** UserWriteTRQueue;
		Flash_Transaction_Queue** GCReadTRQueue;
		Flash_Transaction_Queue** GCWriteTRQueue;
		Flash_Transaction_Queue** GCEraseTRQueue;
		Flash_Transaction_Queue** MappingReadTRQueue;
		Flash_Transaction_Queue** MappingWriteTRQueue;

		sim_time_type* shared_total_time;
		sim_time_type* alone_total_time;
		unsigned long long* total_count;
		unsigned int stream_count;
		void estimate_alone_time(NVM_Transaction_Flash* transaction, Flash_Transaction_Queue* queue);
		void estimate_shared_time(NVM_Transaction_Flash* transaction, Flash_Transaction_Queue* queue);

		bool service_read_transaction(NVM::FlashMemory::Flash_Chip* chip);
		bool service_write_transaction(NVM::FlashMemory::Flash_Chip* chip);
		bool service_erase_transaction(NVM::FlashMemory::Flash_Chip* chip);
		// add function
		void service_transaction(NVM::FlashMemory::Flash_Chip* chip);
	};
}

#endif // TSU_OUTOFORDER_H
