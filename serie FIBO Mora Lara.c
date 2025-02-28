#include <stdio.h>

int main() { 
    // primero todas la variables necesarias
    int posi;      // Posición que se pide en la serie
    double fibo;   // Valor de la serie (se usa long double para evitar desbordamiento en valores grandes)
    double pos1, pos2;  // posicion 1 y posiccion 2 de la serie de fibonacci
    int i;

    // operacions de posiciones dependientes de posi... osea 'posi' es la variable independiente de 'pos1 y pos2'
    pos1 = 0;  // F(0) = 0 ... F(posi)
    pos2 = 1; // F(1) = 1   ... F(posi)
    
    // solicitar al usuario la posición deseada
    printf("----------SERIE DE FIBONACCI----------\n\n");
    printf("Introduzca la posicion de la serie Fibonacci que desea: \n");
    scanf("%i", &posi);

    // validar que la posición no sea negativa
    if (posi < 0) {
        printf("La posicion no puede ser negativa.\n");
        return 1; // Finaliza el programa con error
    }

    if (posi == 0 || posi == 1) {  // posicion 0 o 1 es igual a 1
        fibo = 1;  
    } else {  
        for (i = 1; i < posi; i++) {  
            pos2 = pos2 + pos1;        //pos2 += pos1; se suma el número anterior al actual
            pos1 = pos2 - pos1;     // ajustar pos1 para la siguiente iteración
        }
    }

    fibo = pos2;
    // Mostrar el resultado en consola
    printf("El valor en esa posicion de la serie es: \n  %0.f  \n", fibo);

    return 0;  // Fin del programa
}
  


