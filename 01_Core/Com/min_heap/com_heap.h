#ifndef __COM_HEAP_H__
#define __COM_HEAP_H__

#include <stdint.h>
#include <stdbool.h>

#define HEAP_MAX_SIZE 40


typedef struct {
    int32_t value;          // 存储的值
    uint8_t index;          // 数组索引
} heap_node;

typedef struct {
    heap_node nodes[HEAP_MAX_SIZE];
    int heap_current_size;           // 当前的堆的大小
    int heap_capacity;
    int pos_map[41];                // Maps original_index -> current position in 'nodes' array
    int max_originalIndexLimit;     // Stores the configured limit for original_index
} min_heap;

void heap_init(min_heap* heap, int capacity, int max_originalIndex);
bool heap_insert(min_heap* heap,  uint8_t original_index, int32_t value);
bool heap_peek_min(min_heap* heap, heap_node* get_heap);
bool heap_update(min_heap* heap, uint8_t original_index, int32_t new_value);
bool heap_remove(min_heap* heap, uint8_t original_index);
int heap_find_index(const min_heap *heap, uint8_t target_index);
uint8_t heap_getNodeIndex(min_heap* heap, uint8_t heap_position);

#endif
