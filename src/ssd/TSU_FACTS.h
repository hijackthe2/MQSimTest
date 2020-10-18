#ifndef TSU_FACTS_H
#define TSU_FACTS_H

#include "FTL.h"
#include "TSU_Base.h"
#include "NVM_Transaction_Flash.h"
#include "NVM_PHY_ONFI_NVDDR2.h"

namespace SSD_Components
{
	class FTL;

	class TSU_FACTS : public TSU_Base
	{
	public:
		TSU_FACTS(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI_NVDDR2* NVMController, unsigned int channel_count,
			unsigned int chip_count_per_channel, unsigned int die_count_per_chip, unsigned int plane_count_per_die,
			unsigned int StreamCount, sim_time_type WriteReasonableSuspensionTimeForRead,
			sim_time_type EraseReasonableSuspensionTimeForRead, sim_time_type EraseReasonableSuspensionTimeForWrite,
			bool EraseSuspensionEnabled, bool ProgramSuspensionEnabled);
		~TSU_FACTS();
		void Prepare_for_transaction_submit();
		void Submit_transaction(NVM_Transaction_Flash* transaction);
		void Schedule();

		void Start_simulation();
		void Validate_simulation_config();
		void Execute_simulator_event(MQSimEngine::Sim_Event* event);
		void handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash* transaction);
		void Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter);
	private:
		Flash_Transaction_Queue** UserTRQueue;
		Flash_Transaction_Queue** GCReadTRQueue;
		Flash_Transaction_Queue** GCWriteTRQueue;
		Flash_Transaction_Queue** GCEraseTRQueue;
		Flash_Transaction_Queue** MappingReadTRQueue;
		Flash_Transaction_Queue** MappingWriteTRQueue;

		Flash_Transaction_Queue* buffer;
		sim_time_type* estimated_shared_time;
		sim_time_type* estimated_alone_time;

		unsigned int stream_count;
		sim_time_type* buffer_total_time;
		sim_time_type* queue_total_time;
		sim_time_type* backend_total_time;
		sim_time_type* shared_total_time;
		sim_time_type* alone_total_time;
		sim_time_type* shared_read_total_time;
		sim_time_type* alone_read_total_time;
		sim_time_type* shared_write_total_time;
		sim_time_type* alone_write_total_time;
		unsigned long long* total_count;
		unsigned long long* read_total_count;
		unsigned long long* write_total_count;
		double* slowdown;

		void buffering(NVM_Transaction_Flash* transaction, Flash_Transaction_Queue* buffer, Flash_Transaction_Queue* queue);
		void fairness_scheduling(Flash_Transaction_Queue* buffer, Flash_Transaction_Queue** queue, sim_time_type* shared_time,
			sim_time_type* alone_time);
		void estimate_time(NVM_Transaction_Flash* transaction, Flash_Transaction_Queue* user_queue);

		bool service_read_transaction(NVM::FlashMemory::Flash_Chip* chip);
		bool service_write_transaction(NVM::FlashMemory::Flash_Chip* chip);
		bool service_erase_transaction(NVM::FlashMemory::Flash_Chip* chip);
		void service_transaction(NVM::FlashMemory::Flash_Chip* chip);
	};
}

#endif // TSU_FACTS_H
