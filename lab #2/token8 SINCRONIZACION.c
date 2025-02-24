#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

struct timeval stop, start;
unsigned long tiempo; // Tiempo en milisegundos
unsigned long clave = 0x000000E1;
unsigned long tokenLocal;
unsigned long tokenIngresado;

void imprimeCuadrado() {
    unsigned char dato = 219;
    int i = 0;
    for (i = 1; i <= 288; i++) {
        printf("%c", dato);
        if (i % 24 == 0) {
            printf("\n");
        }
    }
}


int main() {
    printf("\n\nPulse enter para volver a intentar...");
    getchar();
    imprimeCuadrado();
    gettimeofday(&start, NULL);
    
    while(1){
    	
    tokenIngresado=0;
    	
    printf("Ingrese su TOKEN>>> ");
    scanf("%x", &tokenIngresado);
    gettimeofday(&stop, NULL);
    // Calcula el tiempo transcurrido en milisegundos
    tiempo = ((stop.tv_sec - start.tv_sec) * 1000000 + stop.tv_usec - start.tv_usec) / 1000;
    // Calcula el número de intervalos de 30 segundos (30000 ms)
    // Para tiempos menores a 30000 ms, interval será 0; para 30000-59999 ms, 1; etc.
    unsigned long interval = tiempo / 30000;
    // Calcula el operando a usar: 0, 30000, 60000, etc.
   // unsigned long operando = interval * 30;
 
unsigned long operando=0; // LO NECESITO EN 0 PARA TOKEN LOCAL FIRST OPTION = PASSWORD
unsigned long i;

for (i = 1; i <= interval; i++) {
    operando = operando + 300; // Multiplica el operando por 30 en cada iteración
}
	
    // Si operando es 0 (primer intervalo), el token es la misma clave.
    tokenLocal = clave ^ operando;
   
    if ((tokenLocal/tokenIngresado)==1) {
    	
        printf("Token correcto. ¡Acceso permitido!\n");
        printf("Tiempo transcurrido: %lu ms\n", tiempo);
        printf("Intervalo (cada 30s): %lu\n", interval);
        printf("Clave %08lX\n", clave);
        printf("Token Local %08lX\n", tokenLocal);

        
        printf("\n\nAHORA PRESIONE ENTER PARA TERMINAR PROGRAMA\n\n", tokenLocal);        
        getchar();
        return 0;
    } else {
        printf("Token incorrecto. ¡Acceso denegado!\n");
        printf("Tiempo: %lu ms\n", tiempo);
        printf("Intervalo (cada 30s): %lu\n", interval);
        printf("Clave %08lX\n", clave);
        printf("Token Local %08lX\n", tokenLocal);
        tokenIngresado=0;
        system("pause");
    }
   
    	
	}

}
