#ifndef __COM_HEAP_H__
#define __COM_HEAP_H__

#include <stdint.h>
#include <stdbool.h>

#define HEAP_MAX_SIZE 40


typedef struct {
    int32_t value;          // 存储的值
    uint8_t index;          // 数组索引
} heap_dataType;

typedef struct {
    heap_dataType nodes[HEAP_MAX_SIZE];
    int heap_current_size;      // 当前的堆的大小
    // int heap_capacity;
} min_heap;

void heap_init(min_heap* heap);
bool heap_insert(min_heap* heap, int32_t value, uint8_t index);
bool heap_remove_root(min_heap *heap, int32_t *value, uint8_t *index);
bool heap_peek_root(min_heap* heap, int32_t *value, uint8_t *index);
bool heap_update(min_heap* heap, uint8_t target_index, int32_t new_value);
bool heap_remove(min_heap* heap, uint8_t target_index);
int heap_find_index(const min_heap *heap, uint8_t target_index);
uint8_t heap_getNodeIndex(min_heap* heap, uint8_t heap_position);

#endif
