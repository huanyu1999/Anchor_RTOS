#include "dw_sort.h"
#include "elog.h"

static inline int node_less(tag_hashNode_t *a, tag_hashNode_t *b);
static void heap_swap(distance_manager_t *mgr, uint16_t i, uint16_t j);
static void heap_shift_up(distance_manager_t *mgr, uint16_t idx);
static void heap_shift_down(distance_manager_t *mgr, uint16_t idx);
static void node_insert_new(distance_manager_t *mgr, tag_hashNode_t *node);
static void node_update_distance(distance_manager_t *mgr, 
                                 tag_hashNode_t *node,
                                 uint32_t new_dis);
static void node_remove(distance_manager_t *mgr, tag_hashNode_t *node);
static void disManager_DebugPrint(distance_manager_t *mgr);

/**
  * @brief  标签距离更新与插入
  * @param  uint8_t tag_id : 本次获取到的标签id
  * @param  int32_t dis    : 本次获取到的距离
  * @param  uint32_t ticks : 本次TWR成功的tick
  * @retval None
  */





/* 最小堆实现 */
static inline int node_less(tag_hashNode_t *a, tag_hashNode_t *b)
{
    return (a->distance < b->distance);
}

static void heap_swap(distance_manager_t *mgr, uint16_t i, uint16_t j)
{
    tag_hashNode_t *tmp = mgr->heap[i];
    mgr->heap[i] = mgr->heap[j];
    mgr->heap[j] = tmp;

    mgr->heap[i]->heap_index = i;
    mgr->heap[j]->heap_index = j;
}

static void heap_shift_up(distance_manager_t *mgr, uint16_t idx)
{
    while (idx > 0)
    {
        uint16_t parent = (idx - 1) / 2;
        if (!node_less(mgr->heap[idx], mgr->heap[parent]))
        {
            break;
        }
        heap_swap(mgr, idx, parent);
        idx = parent;
    }
}

static void heap_shift_down(distance_manager_t *mgr, uint16_t idx)
{
    while (1)
    {
        uint16_t left = idx * 2 + 1;
        uint16_t right = left + 1;
        uint16_t smallest = idx;

        if (left < mgr->heap_size && node_less(mgr->heap[left], mgr->heap[smallest]))
        {
            smallest = left;
        }

        if (right < mgr->heap_size && node_less(mgr->heap[right], mgr->heap[smallest]))
        {
            smallest = right;
        }
        
        if (smallest == idx)
        {
            break;
        }
        
        heap_swap(mgr, idx, smallest);
        idx = smallest;
    }

}

static void node_insert_new(distance_manager_t *mgr, tag_hashNode_t *node)
{
    uint16_t idx = mgr->heap_size++;
    mgr->heap[idx] = node;
    node->heap_index = idx;
    heap_shift_up(mgr, idx);
}

static void node_update_distance(distance_manager_t *mgr, 
                                 tag_hashNode_t *node,
                                 uint32_t new_dis)
{
    uint32_t old = node->distance;
    node->distance = new_dis;

    if (new_dis < old)
    {
        heap_shift_up(mgr, node->heap_index);
    }
    else
    {
        heap_shift_down(mgr, node->heap_index);
    }
}

static void node_remove(distance_manager_t *mgr, tag_hashNode_t *node)
{
    uint16_t idx = node->heap_index;
    uint16_t last = mgr->heap_size - 1;

    if (idx != last)
    {
        mgr->heap[idx] = mgr->heap[last];
        mgr->heap[idx]->heap_index = idx;
        heap_shift_down(mgr, idx);
        heap_shift_up(mgr, idx);
    }

    HASH_DEL(mgr->hash, node);
    vPortFree(node);
}

/**
  * @brief  距离管理，更新或者插入距离
  * @param  distance_manager_t *mgr ：os 当前tick
  * @param  uint16_t id,  : 要更新的标签距离
  * @param  uint32_t dis  ：要插入的标签距离
  * @param  uint32_t now_tick ：要插入的标签测距成功时间
  * @retval None
  */
void disManager_update(distance_manager_t *mgr,
                        uint16_t id,
                        uint32_t dis,
                        uint32_t now_tick)
{
    tag_hashNode_t *node = NULL;

    HASH_FIND(hh, mgr->hash, &id, sizeof(node->tag_id), node);// hash table 查找的结果有问题，每次执行的操作都是insert，修改key类型长度，目前正常

    if (node) 
    {
        node->last_updateTick = now_tick;
        node_update_distance(mgr, node, dis);
        // log_d("update-> id=%u dis=%u tick=%u", node->tag_id, node->distance, node->last_updateTick);
    }
    else
    {
        if (mgr->heap_size >= MAX_NODE_NUM)
        {
            return ;
        }
        node = pvPortMalloc(sizeof(tag_hashNode_t));
        if (!node)
        {
            return ;
        }
        node->tag_id = id;
        node->distance = dis;
        node->last_updateTick = now_tick;
        HASH_ADD(hh, mgr->hash, tag_id, sizeof(node->tag_id), node);
        node_insert_new(mgr, node);
        log_d("insert-> id=%u dis=%u tick=%u", node->tag_id, node->distance, node->last_updateTick);
    }
}

void disManager_purgeExpired(distance_manager_t *mgr, uint32_t now_tick)
{
    tag_hashNode_t *node, *tmp;

    HASH_ITER(hh, mgr->hash, node, tmp)
    {
        if ((now_tick - node->last_updateTick) > DIST_EXPIRE_TICK)
        {
            node_remove(mgr, node);
            // disManager_DebugPrint(mgr);
            log_d("remove id=%u heap_index=%u", node->tag_id, node->heap_index);
        }
    }
}

tag_hashNode_t *disManager_getMin(distance_manager_t *mgr)
{
    if (mgr->heap_size == 0) 
    {
        return NULL;
    }

    return mgr->heap[0];
}

static void disManager_DebugPrint(distance_manager_t *mgr)
{
    if (mgr->heap_size == 0) {
        log_d("heap empty");
        return;
    }

    log_d("heap_size=%u", mgr->heap_size);
    log_d("Heap order (min at top):");
    for (uint16_t i = 0; i < mgr->heap_size; i++)
    {
        tag_hashNode_t *node = mgr->heap[i];
        log_d("idx=%u id=%u dis=%u last_tick=%u", 
                i, node->tag_id, node->distance, node->last_updateTick);
    }

    log_d("Hash table content:\n");
    tag_hashNode_t *node, *tmp;
    HASH_ITER(hh, mgr->hash, node, tmp) 
    {
        log_d("id=%u distance=%u last_tick=%u heap_index=%u", 
                node->tag_id, node->distance, node->last_updateTick, node->heap_index);
    }
}

// 将32位整数分解为4个8位整数
void split32to8(int32_t value, uint8_t *bytes)
{
    bytes[0] = (value >> 24) & 0xFF; // 高8位
    bytes[1] = (value >> 16) & 0xFF; // 次高8位
    bytes[2] = (value >> 8) & 0xFF;  // 次低8位
    bytes[3] = value & 0xFF;         // 低8位
}

// 将4个8位整数组合成1个32位整数
uint32_t combine8to32(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0] << 24) | 
           ((uint32_t)bytes[1] << 16) | 
           ((uint32_t)bytes[2] << 8)  | 
           (uint32_t)bytes[3];
}
