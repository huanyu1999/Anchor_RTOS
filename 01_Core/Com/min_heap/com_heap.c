// #include "com_heap.h"
// #include "dw_instance.h"
// // #include "usart.h"
// #include <time.h>
// #include <elog.h>

// static void swap_nodes(min_heap *heap, int pos1, int pos2); 
// static void heapify_up(min_heap *heap, int pos);
// static void heapify_down(min_heap *heap, int pos);

// /**
//   * @brief  heap 初始化，初始堆中没有元素，大小为空
//   * @param  min_heap* heap 被操作的堆
//   * @retval None
//   */
// void heap_init(min_heap* heap, int capacity, int max_originalIndex)
// {
//     heap->heap_current_size = 0;  
//     heap->heap_capacity = capacity;
//     heap->max_originalIndexLimit = max_originalIndex;

//     for (int i = 0; i < HEAP_MAX_SIZE; i++)
//     {
//         heap->nodes[i].value = 0;
//         heap->nodes[i].id = 0;
//     }

//     for (int i = 0; i < 41; i++)
//     {
//         heap->pos_map[i] = -1;
//     }
// }

// /**
//   * @brief  heap 插入一个元素
//   * @param  min_heap* heap 被操作的堆
//   * @param  int32_t value  被插入元素的值
//   * @param  uint8_t original_index  被插入元素在原始数组中的索引
//   * @retval false 插入失败
//   * @retval true  插入成功
//   */
// bool heap_insert(min_heap* heap, uint8_t original_index, int32_t value)
// {
//     if (!heap || heap->heap_current_size >= heap->heap_capacity)
//     {
//         log_d("Heap is full or invalid");
//         return false;   // Heap is full or invalid
//     }

//     if (original_index > heap->max_originalIndexLimit)
//     {
//         log_d("Invalid original index");
//         return false;   // Invalid original index
//     }

//     if (heap->pos_map[original_index] != -1)
//     {
//         log_d("Element already exists");
//         return false;   // Element already exists
//     }

//     int cur_index = heap->heap_current_size;                // Add the new node to the end of the heap array
//     heap->nodes[cur_index].value = value;     // 将元素插入到最底部
//     heap->nodes[cur_index].id = original_index;
//     heap->pos_map[original_index] = cur_index;              // pos_map映射当前插入元素的位置
//     heap->heap_current_size++;                              // 容量加一

//     heapify_up(heap, cur_index);                            // 上浮操作，保证最小堆的特性
//     return true;
// }

// /**
//   * @brief  heap 更新已有节点的值
//   * @param  min_heap* heap 被操作的堆
//   * @param  uint8_t original_index 更新的目标数组索引
//   * @param  int32_t new_value    更新的新值
//   * @retval false 更新失败
//   * @retval true 更新成功
//   */
// bool heap_update(min_heap* heap, uint8_t original_index, int32_t new_value)
// {
//     if (!heap) 
//     {
//         log_d("heap not vaild");
//         return false;
//     }

//     if (original_index >= heap->max_originalIndexLimit)
//     {
//         log_d("Invalid original index");
//         return false;
//     }

//     int heap_pos = heap->pos_map[original_index];       // pos_map 映射当前需要更新node对应在堆中位置
    
//     if (heap_pos == -1)
//     {
//         log_d("Element not exist");
//         return false;
//     }

//     int32_t old_value = heap->nodes[heap_pos].value;
//     heap->nodes[heap_pos].value = new_value;
    
//     if (new_value < old_value) 
//     {
//         heapify_up(heap, heap_pos);              
//     }
//     else
//     {
//         heapify_down(heap, heap_pos);
//     }
    
//     return true;
// }

// /**
//   * @brief  heap 删除一个节点
//   * @param  min_heap* heap 被操作的堆
//   * @param  uint8_t original_index 要删除的目标数组索引
//   * @retval None
//   */
// bool heap_remove(min_heap* heap, uint8_t original_index)
// {
//     if (!heap || original_index > heap->max_originalIndexLimit)
//     {
//         return false;
//     }

//     int heap_pos = heap->pos_map[original_index];    // 找到堆中对应的位置
//     if (heap_pos == -1)
//     {
//         return false;
//     }

//     // 执行删除操作，将堆中最后一个节点跟要删除的节点替换，目的是为了保证最小堆的特性，最小堆必定是一个完全二叉树
//     swap_nodes(heap, heap_pos, heap->heap_current_size - 1);
//     heap->heap_current_size--;                      // 减少堆的大小，达到删除的目的 
//     heap->pos_map[original_index] = -1;             // 标志该元素已经被删除

//     if ((heap_pos > 0) && (heap->nodes[heap_pos].value < heap->nodes[(heap_pos - 1) / 2].value))
//     {
//         heapify_up(heap, heap_pos);                 // 删除操作后，被替换的堆位置的值如果比其父节点小，那么就上浮操作
//     }
//     else 
//     {
//         heapify_down(heap, heap_pos);               // 否则下沉操作
//     }

//     return true;
// }

// bool heap_peek_min(min_heap* heap, heap_node* get_heap) 
// {
//     if (!heap || heap->heap_current_size == 0) 
//     {
//         return false;
//     }
//     // value = &heap->nodes[0].value;
//     // index = &heap->nodes[0].index;
//     if (heap)
//     {
//         *get_heap = heap->nodes[0];
//     }
//     return true;
// }

// uint8_t heap_getNodeIndex(min_heap* heap, uint8_t heap_position)
// {
//     uint8_t x = heap->nodes[heap_position].id;
//     return  x;
// }

// /**
//   * @brief  heap 上浮操作
//   * @param  min_heap *heap 被操作的最小堆指针
//   * @param  int pos1 交换的第一个元素在堆中位置
//   * @param  int pos2 交换的第二个元素在堆中位置
//   * @retval None
//   */
// static void swap_nodes(min_heap *heap, int pos1, int pos2)
// {
//     if (pos1 == pos2) { return; }

//     // Update position map first
//     heap->pos_map[heap->nodes[pos1].id] = pos2;
//     heap->pos_map[heap->nodes[pos2].id] = pos1;

//     // Swap node data
//     heap_node temp = heap->nodes[pos1];
//     heap->nodes[pos1] = heap->nodes[pos2];
//     heap->nodes[pos2] = temp;
// }

// /**
//   * @brief  heap 上浮操作
//   * @param  min_heap *heap 被操作的最小堆指针
//   * @param  int pos  被操作的元素在堆中的位置
//   * @retval None
//   */
// static void heapify_up(min_heap *heap, int pos)
// {
//     if (pos <= 0 || pos >= heap->heap_current_size)
//     {
//         return;
//     }

//     int parent_pos = (pos - 1) / 2;           // 获取该节点的父节点
//     while (pos > 0 && heap->nodes[pos].value < heap->nodes[parent_pos].value)
//     {   
//         swap_nodes(heap, pos, parent_pos);
//         pos = parent_pos;
//         if (pos > 0)
//         {
//             parent_pos = (pos - 1) / 2;
//         }
//         else 
//         {
//             break;
//         }
//     }
// }

// /**
//   * @brief  heap 下沉操作
//   * @param  min_heap *heap 被操作的最小堆指针
//   * @param  uint8_t index  被操作的元素在堆中的索引
//   * @retval None
//   */
// static void heapify_down(min_heap *heap, int pos)
// {
//     int cur_pos = pos;

//     while (1)
//     {
//         int left  = 2 * cur_pos + 1;       // 获取左子节点
//         int right = 2 * cur_pos + 2;      // 获取右子节点
//         int smallest = cur_pos; 
//         if ((left < heap->heap_current_size) && (heap->nodes[left].value < heap->nodes[smallest].value))
//         {
//             smallest = left;
//         }
//         if ((right < heap->heap_current_size) && (heap->nodes[right].value < heap->nodes[smallest].value))
//         {
//             smallest = right;
//         }
    
//         if (smallest != cur_pos)
//         {
//             swap_nodes(heap, cur_pos, smallest);
//             cur_pos = smallest;
//         }
//         else
//         {
//             break;
//         }
//     }
// }

// /**
//   * @brief  heap 查找元素对应在堆中的位置
//   * @param  const min_heap *heap 被操作的最小堆指针
//   * @param  uint8_t target_index  被查找元素在数组中的索引
//   * @retval i 被查找元素在堆中的位置
//   * @retval -1 被查找的元素不在堆中
//   */
// int heap_find_index(const min_heap *heap, uint8_t target_index)
// {   
//     return heap->pos_map[target_index];
// }
