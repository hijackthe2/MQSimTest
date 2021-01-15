#include "TSU_SLUnit.h"

namespace SSD_Components
{
	TSU_SLUnit::TSU_SLUnit(unsigned int _total_capacity, const unsigned int _stream_count):
		total_capacity(_total_capacity), stream_count(_stream_count), total_serviced(0)
	{
		buffer = new Flash_Transaction_Queue[stream_count];
		left_buffer_but_not_serviced = new unsigned int[stream_count];
		limited_capacity = new unsigned int[stream_count];
		release_order.resize(stream_count);
		unsigned int avg_capacity = (unsigned int)((float)total_capacity / stream_count + 0.5);
		for (unsigned int stream_id = 0; stream_id < stream_count; ++stream_id)
		{
			left_buffer_but_not_serviced[stream_id] = 0;
			limited_capacity[stream_id] = avg_capacity;
			release_order[stream_id] = stream_id;
		}
	}
	TSU_SLUnit::~TSU_SLUnit()
	{
		delete[] buffer;
		delete[] left_buffer_but_not_serviced;
		delete[] limited_capacity;
	}

	Flash_Transaction_Queue* TSU_SLUnit::get_buffer()
	{
		return this->buffer;
	}

	Flash_Transaction_Queue* TSU_SLUnit::get_buffer(stream_id_type stream_id)
	{
		return &this->buffer[stream_id];
	}

	void TSU_SLUnit::buffering(NVM_Transaction_Flash* transaction)
	{
		buffer[transaction->Stream_id].push_back(transaction);
	}

	std::queue<NVM_Transaction_Flash*> TSU_SLUnit::releasing()
	{
		std::queue<NVM_Transaction_Flash*> q;
		for (unsigned int i = 0; i < stream_count; ++i)
		{
			stream_id_type stream_id = release_order[i];
			while (buffer[stream_id].size() && left_buffer_but_not_serviced[stream_id] < limited_capacity[stream_id])
			{
				NVM_Transaction_Flash* transaction = buffer[stream_id].front(); buffer[stream_id].pop_front();
				q.push(transaction);
				++left_buffer_but_not_serviced[stream_id];
			}
		}
		return q;
	}

	void TSU_SLUnit::releasing(Flash_Transaction_Queue** queue)
	{
		for (unsigned int i = 0; i < stream_count; ++i)
		{
			stream_id_type stream_id = release_order[i];
			while (buffer[stream_id].size() && left_buffer_but_not_serviced[stream_id] < limited_capacity[stream_id])
			{
				NVM_Transaction_Flash* transaction = buffer[stream_id].front(); buffer[stream_id].pop_front();
				queue[transaction->Address.ChannelID][transaction->Address.ChipID].push_back(transaction);
				++left_buffer_but_not_serviced[stream_id];
			}
		}
	}

	void TSU_SLUnit::feedback(const stream_id_type stream_id)
	{
		--left_buffer_but_not_serviced[stream_id];
		++total_serviced;
		if (total_serviced < total_capacity) return;
		total_serviced = 0;
		// get minimum and maximum buffer size
		size_t min_buffer_size = INT_MAX, max_buffer_size = 0, total_buffer_size = 0;
		for (unsigned int stream_id = 0; stream_id < stream_count; ++stream_id)
		{
			if (min_buffer_size > buffer[stream_id].size()) min_buffer_size = buffer[stream_id].size();
			if (max_buffer_size < buffer[stream_id].size()) max_buffer_size = buffer[stream_id].size();
			total_buffer_size = buffer[stream_id].size();
			release_order[stream_id] = stream_id;
		}
		// ascending order by buffer size
		sort(release_order.begin(), release_order.end(), [=](stream_id_type a, stream_id_type b) {return buffer[a].size() < buffer[b].size(); });
		assign_capacity(total_capacity, (unsigned int)total_buffer_size, 0, stream_count - 1);
		// ascending order by limited capacity
		sort(release_order.begin(), release_order.end(), [=](stream_id_type a, stream_id_type b) {return limited_capacity[a] < limited_capacity[b]; });
	}

	void TSU_SLUnit::assign_capacity(unsigned int left_capacity, unsigned int left_buffer_size, unsigned int start, unsigned int end)
	{
		if (start > end || start < 0 || end >= stream_count) return;
		int cnt = end - start + 1;
		unsigned int avg_capacity = (unsigned int)((float)left_capacity / cnt + 0.5);
		if (buffer[release_order[start]].size() >= avg_capacity || buffer[release_order[end]].size() <= avg_capacity) // every workload can get avg_capacity
		{
			for (unsigned int i = start; i <= end; ++i)
			{
				limited_capacity[release_order[i]] = avg_capacity;
			}
		}
		else if (left_buffer_size <= left_capacity) // workloads with larger buffer size are prioritized to assign
		{
			unsigned int i = end;
			while (i >= start && buffer[release_order[i]].size() >= avg_capacity)
			{
				unsigned int size = (unsigned int)buffer[release_order[i]].size();
				limited_capacity[release_order[i]] = size;
				left_capacity -= size;
				left_buffer_size -= size;
				--i;
			}
			assign_capacity(left_capacity, left_buffer_size, start, i);
		}
		else // workloads with smaller buffer size are prioritized to assign
		{
			unsigned int i = start;
			while (i <= end && buffer[release_order[i]].size() <= avg_capacity)
			{
				unsigned int size = (unsigned int)buffer[release_order[i]].size();
				limited_capacity[release_order[i]] = size;
				left_capacity -= size;
				left_buffer_size -= size;
				++i;
			}
			assign_capacity(left_capacity, left_buffer_size, i, end);
		}
	}
}