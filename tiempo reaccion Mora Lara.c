#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void countdown(int seconds) {
	int i;
	i =seconds;
	
    for (i; i > 0; i--) {   //contar hacia atras
        printf("%d...", i); 
        fflush(stdout);  //comentar
        struct timespec ts = {1, 0}; // 1 segundo
        nanosleep(&ts, NULL);  // duerma o quite el programa
    }
    printf("Go!\nContando...\n");
}

int main() {
    printf("Listo para empezar...\n");
    countdown(5);
    
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    getchar(); // espera a que el usuario presione una tecla
    clock_gettime(CLOCK_MONOTONIC, &end);
    
      //////////////////////////////medir respuesta en milisegundos, regla de 3////////////
    double reaction_time = (end.tv_sec - start.tv_sec) * 1000.0 + (end.tv_nsec - start.tv_nsec) / 1000000.0;
    printf("Su tiempo de reaccion fue: %.0lf ms\n", reaction_time);
    
    return 0;
}

