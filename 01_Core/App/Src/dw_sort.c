#include <stdio.h>
#include "dw_sort.h"
#include "elog.h"

static uint8_t hash_node_pool_buf[ HASH_NODE_NUM * sizeof(tag_hashNode_t) ];
static mem_pool_t hash_node_pool;

static inline int node_less(tag_hashNode_t *a, tag_hashNode_t *b);
static void heap_swap(distance_manager_t *mgr, uint16_t i, uint16_t j);
static void heap_shift_up(distance_manager_t *mgr, uint16_t idx);
static void heap_shift_down(distance_manager_t *mgr, uint16_t idx);
static void node_insert_new(distance_manager_t *mgr, tag_hashNode_t *node);
static void node_update_distance(distance_manager_t *mgr, 
                                 tag_hashNode_t *node,
                                 uint32_t new_dis);
static void node_remove(distance_manager_t *mgr, tag_hashNode_t *node);

static void mempool_init(mem_pool_t *mp, void *buf, size_t blk_size, uint32_t cnt);
static void *mempoll_alloc(mem_pool_t *mp);
static void mempool_free(mem_pool_t *mp, void *ptr);

void disManager_memPoolInit(void)
{
    mempool_init(&hash_node_pool, hash_node_pool_buf, sizeof(tag_hashNode_t), HASH_NODE_NUM);
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
    // the hash table also is an array
    HASH_FIND(hh, mgr->hash, &id, sizeof(node->tag_id), node);// hash table 查找，如果找到就更新距离和tick，如果没有找到就插入新节点

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
            // node = pvPortMalloc(sizeof(tag_hashNode_t));
            node = mempoll_alloc(&hash_node_pool);
            if (!node)
            {
                return ;
            }
            node->tag_id = id;
            node->distance = dis;
            node->last_updateTick = now_tick;
            HASH_ADD(hh, mgr->hash, tag_id, sizeof(node->tag_id), node);        // create an node, insert to hashtable
            node_insert_new(mgr, node);
            log_i("+ tag %u dis=%.2fm (online:%u)", node->tag_id, (float)node->distance / 1000.0f, mgr->heap_size);
    }
}

void disManager_purgeExpired(distance_manager_t *mgr, uint32_t now_tick)
{
    tag_hashNode_t *node, *tmp;

    HASH_ITER(hh, mgr->hash, node, tmp)
    {
        if ((now_tick - node->last_updateTick) > DIST_EXPIRE_TICK)
        {
            uint8_t removed_id = node->tag_id;
            node_remove(mgr, node);
            log_i("- tag %u timeout (online:%u)", removed_id, mgr->heap_size);
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

uint16_t disManager_getNodeNum(distance_manager_t *mgr)
{
    return mgr->heap_size;
}

void disManager_logSummary(distance_manager_t *mgr)
{
    if (mgr->heap_size == 0)
    {
        log_i("online:0");
        return;
    }

    tag_hashNode_t *min_node = mgr->heap[0];
    log_i("online:%u min:tag-%u %.2fm",
           mgr->heap_size, min_node->tag_id,
           (float)min_node->distance / 1000.0f);

    static char buf[128];
    int pos = 0;
    for (uint16_t i = 0; i < mgr->heap_size && pos < (int)sizeof(buf) - 16; i++)
    {
        tag_hashNode_t *n = mgr->heap[i];
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "%u:%.2f ", n->tag_id, (float)n->distance / 1000.0f);
    }
    if (pos > 0)
    {
        // log_d("[tags] %s", buf);
    }
}

/******************************************************** 最小堆实现*********************************************************/
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

    if (idx != last)            // if current node is not the last node
    {
        mgr->heap[idx] = mgr->heap[last];
        mgr->heap[idx]->heap_index = idx;
        heap_shift_down(mgr, idx);
        heap_shift_up(mgr, idx);
    }

    HASH_DEL(mgr->hash, node);
    mgr->heap_size--;
    // vPortFree(node);
    mempool_free(&hash_node_pool, node);
}

/***********************************************************mem pool******************************************************************* */
static void mempool_init(mem_pool_t *mp, void *buf, size_t blk_size, uint32_t cnt)
{
    uint8_t *p = buf;
    mp->free = NULL;

    for (uint32_t i = 0; i < cnt; i++)
    {
        mem_blk_t *b = (mem_blk_t*) p;
        b->next = mp->free;
        mp->free = b;
        p += blk_size;              // the point jump to next block
    }
}

static void *mempoll_alloc(mem_pool_t *mp)
{
    mem_blk_t *b;
    
    taskENTER_CRITICAL();
    b = mp->free;
    if (b)
    {
        mp->free = b->next;
    }
    taskEXIT_CRITICAL();

    return b;
}

static void mempool_free(mem_pool_t *mp, void *ptr)
{
    mem_blk_t *b = (mem_blk_t*)ptr;

    taskENTER_CRITICAL();
    b->next = mp->free;
    mp->free = b;
    taskEXIT_CRITICAL();
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
