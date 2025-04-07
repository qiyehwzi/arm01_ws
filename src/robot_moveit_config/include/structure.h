#ifndef CONTROL_STRUCTURE_H_
#define CONTROL_STRUCTURE_H_
#include <pthread.h>
#include <time.h>
#include <cstdint>

#define SHM_NAME "/transform_matrix_shm"
#define SHM_SIZE sizeof(SharedData)

typedef struct {
    pthread_mutex_t mutex;       // 互斥锁确保独占访问
    double matrix[4][4];          // 4x4变换矩阵
    short color;                 // 我方颜色
    uint64_t version;            // 版本号
} SharedData;
#endif