#include "com_heap.h"
#include "usart.h"
#include <time.h>

static void swap_nodes(heap_dataType* a, heap_dataType* b); 
static void heapify_up(min_heap *heap, uint8_t index);
static void heapify_down(min_heap *heap, uint8_t index);

/**
  * @brief  heap 初始化，初始堆中没有元素，大小为空
  * @param  min_heap* heap 被操作的堆
  * @retval None
  */
void heap_init(min_heap* heap)
{
    heap->heap_current_size = 0;    // 初始化堆，堆中没有元素
}

/**
  * @brief  heap 插入一个元素
  * @param  min_heap* heap 被操作的堆
  * @param  int32_t value  被插入元素的值
  * @param  uint8_t index  被插入元素在数组中的索引
  * @retval false 插入失败
  * @retval true  插入成功
  */
bool heap_insert(min_heap* heap, int32_t value, uint8_t index)
{
    if (heap->heap_current_size >= HEAP_MAX_SIZE)       // 判断当前堆的大小，超过最大的容量就报错无法插入
    {
        return false;
    }

    heap->nodes[heap->heap_current_size].value = value; // 将元素插入到最底部
    heap->nodes[heap->heap_current_size].index = index;
    heap->heap_current_size++;                          // 容量加一

    heapify_up(heap, heap->heap_current_size - 1);      // 上浮操作，保证最小堆的特性
    return true;
}

bool heap_remove_root(min_heap *heap, int32_t *value, uint8_t *index)
{   
    if (heap->heap_current_size == 0)
    {
        return false;
    }

    if (value != NULL) 
    {
        *value = heap->nodes[0].value;
    }

    if (index != NULL) 
    {
        *index = heap->nodes[0].index;
    }

    heap->heap_current_size--;
    heap->nodes[0] = heap->nodes[heap->heap_current_size];

    heapify_down(heap, 0);
    return true;
}

bool heap_peek_root(min_heap* heap, int32_t *value, uint8_t *index)
{
    if (heap->heap_current_size == 0)
    {
        return false;
    }

    if (value != NULL) 
    {
        *value = heap->nodes[0].value;
    }

    if (index != NULL) 
    {
        *index = heap->nodes[0].index;
    }

    return true;
}

/**
  * @brief  heap 更新已有节点的值
  * @param  min_heap* heap 被操作的堆
  * @param  uint8_t target_index 更新的目标数组索引
  * @param  int32_t new_value    更新的新值
  * @retval false 更新失败
  * @retval true 更新成功
  */
bool heap_update(min_heap* heap, uint8_t target_index, int32_t new_value)
{
    int heap_position = heap_find_index(heap, target_index);
    if (heap_position == -1)
    {
        return false;
    }

    heap->nodes[heap_position].value = new_value;
    
    heapify_up(heap, heap_position);
    heapify_down(heap, heap_position);

    return true;
}

/**
  * @brief  heap 删除一个节点
  * @param  min_heap* heap 被操作的堆
  * @param  uint8_t target_index 要删除的目标数组索引
  * @retval None
  */
bool heap_remove(min_heap* heap, uint8_t target_index)
{
    int heap_position = heap_find_index(heap, target_index);    // 找到堆中对应的位置
    if (heap_position == -1)
    {
        return false;
    }

    heap->nodes[heap_position] = heap->nodes[heap->heap_current_size - 1]; // 执行删除操作，将堆中最后一个节点跟要删除的节点替换，目的是为了保证最小堆的特性，最小堆必定是一个完全二叉树，
    heap->heap_current_size--;  // 减少堆的大小，达到删除的目的 

    if ((heap_position > 0) && (heap->nodes[heap_position].value < heap->nodes[(heap_position - 1) / 2].value))
    {
        heapify_up(heap, heap_position);  // 删除操作后，被替换的堆位置的值如果比其父节点小，那么就上浮操作
    }
    else 
    {
        heapify_down(heap, heap_position);  // 否则下沉操作，
    }

    return true;
}

uint8_t heap_getNodeIndex(min_heap* heap, uint8_t heap_position)
{
    uint8_t x = heap->nodes[heap_position].index;
    return  x;
}

/**
  * @brief  heap 上浮操作
  * @param  heap_dataType* a 交换的第一个元素
  * @param  heap_dataType* b 交换的第二个元素
  * @retval None
  */
static void swap_nodes(heap_dataType* a, heap_dataType* b)
{
    heap_dataType temp = *a;
    *a = *b;
    *b = temp;
}

/**
  * @brief  heap 上浮操作
  * @param  min_heap *heap 被操作的最小堆指针
  * @param  uint8_t position  被操作的元素在堆中的位置
  * @retval None
  */
static void heapify_up(min_heap *heap, uint8_t position)
{
    while (position > 0)
    {   
        uint8_t parent = (position - 1) / 2;           // 获取该节点的父节点
        if (heap->nodes[position].value < heap->nodes[parent].value)
        {
            swap_nodes(&heap->nodes[position], &heap->nodes[parent]);
            position = parent;
        }
        else 
        {
            break;
        }
    }
}

/**
  * @brief  heap 下沉操作
  * @param  min_heap *heap 被操作的最小堆指针
  * @param  uint8_t index  被操作的元素在堆中的索引
  * @retval None
  */
static void heapify_down(min_heap *heap, uint8_t index)
{
    while (1) 
    {
        uint8_t left = 2 * index + 1;       // 获取左子节点
        uint8_t right = 2 * index + 2;      // 获取右子节点
        uint8_t smallest = index;           

        if ((left < heap->heap_current_size) && (heap->nodes[left].value < heap->nodes[smallest].value))
        {
            smallest = left;
        }
        if ((right < heap->heap_current_size) && (heap->nodes[right].value < heap->nodes[smallest].value))
        {
            smallest = right;
        }

        if (smallest != index)
        {
            swap_nodes(&heap->nodes[index], &heap->nodes[smallest]);
            index = smallest;
        }
        else 
        {
            break;
        }
    }
}

/**
  * @brief  heap 查找元素对应在堆中的位置
  * @param  const min_heap *heap 被操作的最小堆指针
  * @param  uint8_t target_index  被查找元素在数组中的索引
  * @retval i 被查找元素在堆中的位置
  * @retval -1 被查找的元素不在堆中
  */
int heap_find_index(const min_heap *heap, uint8_t target_index)
{   
    for (uint8_t i = 0; i < heap->heap_current_size; i++)
    {
        if (heap->nodes[i].index == target_index)
        {
            return i;
        }
    }

    return -1;
}
