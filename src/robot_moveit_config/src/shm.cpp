// #include "data_manager/control/control.h"
#include <thread>
#include <cmath>
#include <fstream>
#include <cstring>
// #include "mining_tank/detector.h"
#include "../include/structure.h"

// /* 初始化共享内存（生产者端） */
// SharedData* init_shared_memory() {
//     int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
//     ftruncate(fd, sizeof(SharedData));

//     SharedData* data = mmap(NULL, sizeof(SharedData), 
//                            PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

//     // 初始化互斥锁（跨进程）
//     pthread_mutexattr_t mutex_attr;
//     pthread_mutexattr_init(&mutex_attr);
//     pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
//     pthread_mutex_init(&data->mutex, &mutex_attr);

//     // 初始化默认值
//     memset(data->matrix, 0, sizeof(double[4][4]));
//     data->color = 0;  // 默认颜色
//     data->version = 0; // 初始化版本号

//     return data;
// }

int main(int argc, char const *argv[])
{
    return 0;
}

/* 原子化写入操作 */
void update_shared_data(SharedData* data, 
    const float new_matrix[4][4],
    short new_color) 
{
    pthread_mutex_lock(&data->mutex);
    memcpy(data->matrix, new_matrix, sizeof(double[4][4]));
    data->color = new_color;
    data->version++; // 更新版本号
    pthread_mutex_unlock(&data->mutex);
}

/* 原子化读取操作 */
void read_shared_data(SharedData* data,
    double out_matrix[4][4],
    short* out_color,
    uint64_t* out_ts) 
{
    pthread_mutex_lock(&data->mutex);
    //memcpy(out_matrix, data->matrix, sizeof(double[4][4]));
    *out_color = data->color;
    //*out_ts = data->version;
    pthread_mutex_unlock(&data->mutex);
}
