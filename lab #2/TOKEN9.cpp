#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

struct timeval stop, start;
unsigned long tiempo;
unsigned long clave = 0x000000E1;
unsigned long tokenLocal = 0xffffffff;
unsigned long tokenIngresado;

void imprimeCuadrado() {
    unsigned char dato = 219;
    for (int i = 1; i <= 288; i++) {
        printf("%c", dato);
        if (i % 24 == 0) printf("\n");
    }
}

int main() {
    printf("\n\nPulse enter para volver a intentar...");
    getchar();
    imprimeCuadrado();
    gettimeofday(&start, NULL);

    while(1) {
        tokenIngresado = 0;
        printf("Ingrese su TOKEN>>> ");
        scanf("%lx", &tokenIngresado);  // Corregido formato
        gettimeofday(&stop, NULL);

        tiempo = ((stop.tv_sec - start.tv_sec) * 1000000 + 
                 (stop.tv_usec - start.tv_usec)) / 1000;
        unsigned long interval = tiempo / 30000;
        unsigned long operando = interval * 30;  // Optimizado

        tokenLocal = clave ^ operando;

        if (tokenIngresado == tokenLocal) {  // Condición corregida
            printf("Token correcto. ¡Acceso permitido!\n");
            printf("Tiempo transcurrido: %lu ms\n", tiempo);
            printf("Clave %08lX\n", clave);
            printf("Token Local %08lX\n", tokenLocal);
        } else {
            printf("\n\nToken incorrectooo. ¡Acceso denegado!\n");
            printf("Tiempo: %lu ms\n", tiempo);
                        printf("Clave %08lX\n", clave);
                                    system("pause");
        }
    }
    return 0;
}
