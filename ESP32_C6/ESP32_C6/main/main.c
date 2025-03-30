#include <stdio.h> //printf function
#include <unistd.h> //Sleep function

void app_main(void)
{
    while (1) {
        sleep(3);//sleeps for 3 seconds
        printf("Hello World\n");
    }
}