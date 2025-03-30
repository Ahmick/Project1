#include <stdio.h> //printf function
#include <unistd.h> //Sleep function

void app_main(void)
{
    while (1) {
        sleep(1);//sleeps for 3 seconds
        printf("Hello World\n");
        printf("I am a ESP32-C6\n");
    }
}