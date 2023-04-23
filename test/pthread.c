#include <stdio.h>
#include <unistd.h>   //调用 sleep() 函数
#include <pthread.h>  //调用 pthread_create() 函数

void *ThreadFun(void *arg)
{
    if (arg == NULL) {
        printf("arg is NULL\n");
    }
    else {
        printf("%s\n", (char*)arg);
    }
    while(1)
	    printf("xxx\n");
    return NULL;
}

int main()
{
    int res;
    char * url = "http://c.biancheng.net";
    //定义两个表示线程的变量（标识符）
    pthread_t myThread1,myThread2;
    //创建 myThread1 线程
    res = pthread_create(&myThread1, NULL, ThreadFun, NULL);
    if (res != 0) {
        printf("线程创建失败");
        return 0;
    }
    sleep(10);  //令主线程等到 myThread1 线程执行完成
#if 0
    //创建 myThread2 线程
    res = pthread_create(&myThread2, NULL, ThreadFun,(void*)url);
    if (res != 0) {
        printf("线程创建失败");
        return 0;
    }
    sleep(5); // 令主线程等到 mythread2 线程执行完成
#endif
    return 0;
}
