#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdint.h>



struct SharedData{
    pthread_mutex_t mutex;       // 互斥锁确保独占访问
    double matrix[4][4];          // 4x4变换矩阵
    short color;                 // 我方颜色
    uint64_t version;            // 版本号
};

#define SHM_NAME "/transform_matrix_shm"
#define SHM_SIZE sizeof(SharedData)


/* 原子化写入（统一矩阵类型为 double） */
void update_shared_data(
    SharedData* data,
    short new_color
) {
    pthread_mutex_lock(&data->mutex); // 安全复制
    data->color = new_color;
    pthread_mutex_unlock(&data->mutex);
}

/* 原子化读取（完整数据拷贝） */
void read_shared_data(
    SharedData* data,
    double out_matrix[4][4],
    uint64_t* out_version
) {
    pthread_mutex_lock(&data->mutex);
    memcpy(out_matrix, data->matrix, sizeof(double[4][4]));
    *out_version = data->version;
    pthread_mutex_unlock(&data->mutex);
}
int main() {
    int fd = -1;
    while(fd == -1) {
        fd = shm_open(SHM_NAME, O_RDWR | O_CREAT, 0666);
        if (fd == -1) {
            std::cerr << "Failed to create shared memory." << std::endl;
        }
    }
    SharedData* shm = (SharedData*)mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    // 更新数据
    short color = 1; // 1 for red, 2 for blue
    update_shared_data(shm, color);

    while(1)
    {
        //读取数据
        double matrix[4][4];
        uint64_t version;
        read_shared_data(shm, matrix, &version);
        std::cout << "Matrix: " << std::endl;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                std::cout << matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    //释放共享内存调用
    munmap(shm, SHM_SIZE);
    return 0;
}
