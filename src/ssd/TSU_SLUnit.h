#ifndef TSU_SLUNIT_H
#define TSU_SLUNIT_H

#include "Flash_Transaction_Queue.h"
#include "NVM_Transaction_Flash.h"
#include <algorithm>
#include <limits.h>
#include <vector>
#include <queue>

namespace SSD_Components
{
	class TSU_SLUnit
	{
	public:
		TSU_SLUnit(unsigned int _total_capacity, const unsigned int _stream_count);
		~TSU_SLUnit();
		Flash_Transaction_Queue* get_buffer();
		Flash_Transaction_Queue* get_buffer(stream_id_type stream_id);
		void buffering(NVM_Transaction_Flash* transaction);
		std::queue<NVM_Transaction_Flash*> releasing();
		void releasing(Flash_Transaction_Queue** queue);
		void feedback(const stream_id_type stream_id);
	private:
		void assign_capacity(unsigned int left_capacity, unsigned int left_buffer_size, unsigned int start, unsigned int end);
		unsigned int total_capacity;
		unsigned int stream_count;
		Flash_Transaction_Queue* buffer;
		unsigned int* left_buffer_but_not_serviced;
		unsigned int* limited_capacity;
		std::vector<stream_id_type> release_order;
		unsigned int total_serviced;
	};
}

#endif // !TSU_SLUNIT_H
