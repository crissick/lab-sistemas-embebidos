#include <stdio.h>
#include <stdlib.h>

int menu;
double Resistencia1;
double Resistencia2;
double VoltajeFuente;
double VoltajeSalida;

void DivisorVoltajeNormal();
void DivisorVoltajeResisDesco();
void DivisorVoltajeFuenteDesco();

int main()
{
    while(1){
        system("cls");
        printf("SELECCIONE UNA OPCION DEL DIVISOR DE VOLTAJE: \n 1) Conocer el voltaje de salida. \n 2) Conocer el valor de las resistencias. \n 3) Conocer el voltaje de la fuente de entrada. \n 4) Salir \n");
        scanf("%i", &menu);
        if(menu > 4)
        {
            printf("Ingrese una opcion valida \n");
            system("pause");
        }
        switch (menu)
               {
        case 1: system("cls");
                printf("Ingrese el valor de la fuente: \n");
                scanf("%lf", &VoltajeFuente);
                printf("Ingrese el valor de la resistencia 1: \n");
                scanf("%lf", &Resistencia1);
                printf("Ingrese el valor de la resistencia 2: \n");
                scanf("%lf", &Resistencia2);
                DivisorVoltajeNormal(VoltajeFuente,Resistencia1,Resistencia2);
            break;

        case 2:system("cls");
               printf("Ingrese el valor de la fuente: \n");
               scanf("%lf", &VoltajeFuente);
               printf("Ingrese el valor de salida : \n");
               scanf("%lf", &VoltajeSalida);
               DivisorVoltajeResisDesco(VoltajeFuente,VoltajeSalida);
            break;

        case 3:system("cls");
               printf("Ingrese el voltaje de salida: \n");
               scanf("%lf", &VoltajeSalida);
               printf("Ingrese el valor de la resistencia 1: \n");
               scanf("%lf", &Resistencia1);
               printf("Ingrese el valor de la resistencia 2: \n");
               scanf("%lf", &Resistencia2);
               DivisorVoltajeFuenteDesco(VoltajeSalida,Resistencia1,Resistencia2);
            break;
        case 4: exit(0);
    }

}
    return 0;
}

void DivisorVoltajeNormal(double Voltajefuente, double res1,double res2)
{
    VoltajeSalida = (Voltajefuente)*((res2)/(res1+res2));
    printf("El voltaje de salida es: %lf V \n", VoltajeSalida);
    system("pause");
    VoltajeFuente=0;
    Resistencia1=0;
    Resistencia2=0;
    VoltajeSalida=0;
}
void DivisorVoltajeResisDesco(double Voltajefuente,double Voltajesalida)
{
    double res1 = (Voltajefuente)-(Voltajesalida);
    double res2 = Voltajesalida;
    printf("El valor de la resistencia 1 es: %lf Ohmios \n El valor de la resistencia 2 es: %lf Ohmios \n", res1,res2);
    system("pause");
    VoltajeFuente=0;
    Resistencia1=0;
    Resistencia2=0;
    VoltajeSalida=0;
}
void DivisorVoltajeFuenteDesco(double Voltajesalida, double res1,double res2)
{
    double Voltajefuente = ((Voltajesalida)/(res2))*((res1)+(res2));
    printf("El voltaje de la fuente es: %lf V \n", Voltajefuente);
    system("pause");
    VoltajeFuente=0;
    Resistencia1=0;
    Resistencia2=0;
    VoltajeSalida=0;
}
