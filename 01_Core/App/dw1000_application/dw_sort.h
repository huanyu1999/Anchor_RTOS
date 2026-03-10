#ifndef __DW_SORT_H__
#define __DW_SORT_H__

#include "cmsis_os.h"
#include "uthash.h"

#define MAX_NODE_NUM 50
#define DIST_EXPIRE_TICK  pdMS_TO_TICKS(2000)
#define HASH_NODE_NUM 60

typedef struct mem_blk {
    struct mem_blk *next;
} mem_blk_t;

typedef struct {
    mem_blk_t *free;
}mem_pool_t;

typedef struct {
    uint8_t tag_id;             // 标签id，锚点编号（key）
    int32_t distance;           // 距离值
    uint32_t last_updateTick;   // 标签最后一次twr时间
    uint16_t heap_index;        // 在最小堆的位置
    UT_hash_handle hh;          // uthash 句柄
} tag_hashNode_t;

typedef struct {
    tag_hashNode_t* hash;       // uthash 头指针
    tag_hashNode_t* heap[MAX_NODE_NUM];  // 用于存储tag_hashNode_t类型指针数组
    uint16_t heap_size;
} distance_manager_t;

void disManager_memPoolInit(void);
void disManager_update(distance_manager_t *mgr,
                        uint16_t id,
                        uint32_t dis,
                        uint32_t now_tick);
void disManager_purgeExpired(distance_manager_t *mgr, uint32_t now_tick);
tag_hashNode_t *disManager_getMin(distance_manager_t *mgr);
uint16_t disManager_getNodeNum(distance_manager_t *mgr);
void split32to8(int32_t value, uint8_t *bytes);
uint32_t combine8to32(const uint8_t *bytes);
#endif
