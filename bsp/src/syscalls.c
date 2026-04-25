#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <stddef.h>
#include "uart.h"

typedef char * caddr_t;
// 解决 _write 报错，将 printf 重定向到你的 UART
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        Putc_UART(ptr[i]); // 调用你之前写的串口发送函数
    }
    return len;
}

// 在你的 C 代码中引用链接脚本定义的边界
extern char _ebss;   // BSS 段结束，即堆开始
extern char _estack; // RAM 结束，即栈顶

caddr_t _sbrk(int incr) {
    static char *heap_end = NULL;
    char *prev_heap_end;

    if (heap_end == NULL) {
        heap_end = &_ebss; // 初始化为 BSS 结束地址
    }

    prev_heap_end = heap_end;

    // 简单的堆栈碰撞检查
    // 如果申请后的堆空间超过了栈底，说明内存耗尽
    if (heap_end + incr > &_estack) {
        // 实际上这里应该预留一点空间给栈，
        // 否则堆完全顶到栈底会导致程序崩溃
        return (caddr_t)-1; 
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}

// 补齐其他常见的空桩函数
int _close(int file) { return -1; }
int _fstat(int file, struct stat *st) { return 0; }
int _isatty(int file) { return 1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _read(int file, char *ptr, int len) { return 0; }
void _exit(int status) { while(1); }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }