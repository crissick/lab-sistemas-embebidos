#include <stdio.h>
#include <stdlib.h>
#include <time.h>

    void start_counter(int max_seconds) {
   // printf("Presione Enter para iniciar el contador...\n");
    //getchar(); // Espera a que el usuario presione una tecla
    
    struct timespec start, current;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    int counterspot = 0;
    while (counterspot < max_seconds) { // Contador hasta max_seconds
        clock_gettime(CLOCK_MONOTONIC, &current);
        counterspot = (current.tv_sec - start.tv_sec);
        
        printf("Tiempo transcurrido: %d s\r", counterspot);
        fflush(stdout);
        
        struct timespec ts = {1, 0};
        nanosleep(&ts, NULL);
    }
    
    printf("\nLlego a tiempo limite.\n", max_seconds);
}



int main() {
    start_counter(200);
    return 0;
}

