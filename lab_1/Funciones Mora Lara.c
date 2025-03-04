#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double VectorCoordsX [10];
double VectorCoordsY [10];
double Pendiente=0,CorteY=0;
double CoefXCuad=0, CoefX=0,Independiente=0;

void ErrorCuadMedio();
void AjusteLineal();
void AjusteCuadratico();
double DeterminanteMatrices();

int main()
{
	int i=0;
    for (i=0; i<=9;i++)
        {
            system("cls");
            printf("INGRESE LAS COORDENADAS DE X%i : \n",i+1);
            scanf("%lf", &VectorCoordsX [i]);
        }
        int c=0;
    for (c=0; c<=9;c++)
        {
            system("cls");
            printf("INGRESE LAS COORDENADAS DE Y%i: \n", i+1);
            scanf("%lf", &VectorCoordsY [i]);
        }
    AjusteLineal(VectorCoordsX,VectorCoordsY);
    AjusteCuadratico(VectorCoordsX,VectorCoordsY);
    ErrorCuadMedio(Pendiente,CorteY,CoefXCuad,CoefX,Independiente);
}
double DeterminanteMatrices(double Matrix[3][3])
{
    double Determinante=0;
    Determinante = ( ((Matrix[0][0])*(Matrix[1][1])*(Matrix[2][2]))/*1*/ +
                     ((Matrix[1][0])*(Matrix[2][1])*(Matrix[0][2]))/*2*/ +
                     ((Matrix[2][0])*(Matrix[0][1])*(Matrix[1][2]))/*3*/ -
                     ((Matrix[0][2])*(Matrix[1][1])*(Matrix[2][0]))/*4*/ -
                     ((Matrix[1][2])*(Matrix[2][1])*(Matrix[0][0]))/*5*/ -
                     ((Matrix[2][2])*(Matrix[0][1])*(Matrix[1][0]))/*6*/
                   );
    return Determinante;
}
void ErrorCuadMedio(double PendienteLineal, double CorteYLineal,double CoefXCuadrado, double CoefXNormal, double TermIndepen)
{
    double ECMLineal=0;double ECMCuadratico=0;
    int d=0;
    for (d=0; d<=9;d++)
    {
        ECMLineal += pow((VectorCoordsY[i])-(((PendienteLineal)*(VectorCoordsX[i]))+CorteYLineal),2);
        ECMCuadratico += pow((VectorCoordsY[i])-(((CoefXCuadrado)*(pow(VectorCoordsX[i],2)))+((CoefXNormal)*(VectorCoordsX[i]))+(TermIndepen)),2);
    }
    if(ECMLineal==ECMCuadratico)
    {
        printf("El ECM es igual para ambos casos, por lo tanto no importa que ecuacion use, se mostrara a continuacion las soliciones: \n");
        printf("Lineal: %.2lf X + %.2lf \n",PendienteLineal,CorteYLineal);
        printf("Cuadratica: %.2lf X^2 + %.2lf X + %.2lf \n",CoefXCuadrado,CoefXNormal,TermIndepen);
    }else if(ECMLineal < ECMCuadratico)
            {
                printf("La ecuacion lineal es la que mejor se ajusta a los puntos dados, la ecuacion es: \n");
                printf(" %.2lf X + %.2lf \n",PendienteLineal,CorteYLineal);
            }else{
                    printf("La ecuacion cuadratica es la que mejor se ajusta a los puntos dados, la ecuacion es: \n");
                    printf(" %.2lf X^2 + %.2lf X + %.2lf \n",CoefXCuadrado,CoefXNormal,TermIndepen);
                 }
}
void AjusteLineal(double CoordsX [10],double CoordsY [10])
{
    double SumatoriaXi=0,SumatoriaYi=0,SumatoriaXiYi=0,SumatoriaXiCuad=0;
    double Determinante=0, DeterminanteX=0,DeterminanteY=0;
    for (int i=0; i<=9;i++)
        {
            SumatoriaXi += CoordsX[i];
            SumatoriaYi += CoordsY[i];
            SumatoriaXiYi += ((CoordsX[i])*(CoordsY[i]));
            SumatoriaXiCuad += pow(CoordsX[i],2);
        }
    Determinante = (pow(SumatoriaXi,2))-((10)*(SumatoriaXiCuad));
    DeterminanteX = ((SumatoriaYi)*(SumatoriaXi))-((10)*(SumatoriaXiYi));
    DeterminanteY = ((SumatoriaXi)*(SumatoriaXiYi))-((SumatoriaYi)*(SumatoriaXiCuad));

    if(Determinante!=0)
        {
            Pendiente = ((DeterminanteX)/(Determinante));
            CorteY = ((DeterminanteY)/(Determinante));
        }else if(DeterminanteX==0 && DeterminanteY==0)
                {
                    printf("La ecuacion lineal tiene infinitas soluciones, los puntos se ajustan perfectamente o con ECM positivo, por tanto, no es posible mostrar UNA SOLA ECUACION. \n");
                    exit(0);
                }else
                    {
                        printf("La ecuacion lineal no tiene solucion, por lo tanto, el ECM sera positivo y por tanto no sera preciso en los puntos. \n");
                        exit(0);
                    }
}
void AjusteCuadratico(double CoordsX [10],double CoordsY [10])
{
    double MatrizCoeficientes[3][3];
    double SumatoriaXi=0, SumatoriaYi=0,SumatoriaXiCuad=0,SumatoriaXiCubo=0, SumatoriaXiTetra=0, SumatoriaXiYi=0, SumatoriaXiCuadYi=0;
    double Determinante=0,DeterminanteX=0,DeterminanteY=0,DeterminanteZ=0;
    for (int i=0; i<=9;i++)
        {
            SumatoriaXi += CoordsX[i];
            SumatoriaYi += CoordsY[i];
            SumatoriaXiCuad += pow(CoordsX[i],2);
            SumatoriaXiCubo += pow(CoordsX[i],3);
            SumatoriaXiTetra += pow(CoordsX[i],4);
            SumatoriaXiYi += ((CoordsX[i])*(CoordsY[i]));
            SumatoriaXiCuadYi += ((pow(CoordsX[i],2))*(CoordsY[i]));
        }

    MatrizCoeficientes[0][0]=SumatoriaXiCuad;   MatrizCoeficientes[0][1]=SumatoriaXi;       MatrizCoeficientes[0][2]=10;
    MatrizCoeficientes[1][0]=SumatoriaXiCubo;   MatrizCoeficientes[1][1]=SumatoriaXiCuad;   MatrizCoeficientes[1][2]=SumatoriaXi;
    MatrizCoeficientes[2][0]=SumatoriaXiTetra;  MatrizCoeficientes[2][1]=SumatoriaXiCubo;   MatrizCoeficientes[2][2]=SumatoriaXiCuad;
    Determinante = DeterminanteMatrices(MatrizCoeficientes);
    if(Determinante==0){
        printf("El determinante principal da cero, no se puede aplicar la regla de Cramer para solucionarlo, ya que tiene mas de una solucion.");
        exit(0);
    }

    MatrizCoeficientes[0][0]=SumatoriaYi;       MatrizCoeficientes[0][1]=SumatoriaXi;     MatrizCoeficientes[0][2]=10;
    MatrizCoeficientes[1][0]=SumatoriaXiYi;     MatrizCoeficientes[1][1]=SumatoriaXiCuad; MatrizCoeficientes[1][2]=SumatoriaXi;
    MatrizCoeficientes[2][0]=SumatoriaXiCuadYi; MatrizCoeficientes[2][1]=SumatoriaXiCubo; MatrizCoeficientes[2][2]=SumatoriaXiCuad;
    DeterminanteX = DeterminanteMatrices(MatrizCoeficientes);

    MatrizCoeficientes[0][0]=SumatoriaXiCuad;   MatrizCoeficientes[0][1]=SumatoriaYi;        MatrizCoeficientes[0][2]=10;
    MatrizCoeficientes[1][0]=SumatoriaXiCubo;   MatrizCoeficientes[1][1]=SumatoriaXiYi;      MatrizCoeficientes[1][2]=SumatoriaXi;
    MatrizCoeficientes[2][0]=SumatoriaXiTetra;  MatrizCoeficientes[2][1]=SumatoriaXiCuadYi;  MatrizCoeficientes[2][2]=SumatoriaXiCuad;
    DeterminanteY = DeterminanteMatrices(MatrizCoeficientes);

    MatrizCoeficientes[0][0]=SumatoriaXiCuad;   MatrizCoeficientes[0][1]=SumatoriaXi;       MatrizCoeficientes[0][2]=SumatoriaYi;
    MatrizCoeficientes[1][0]=SumatoriaXiCubo;   MatrizCoeficientes[1][1]=SumatoriaXiCuad;   MatrizCoeficientes[1][2]=SumatoriaXiYi;
    MatrizCoeficientes[2][0]=SumatoriaXiTetra;  MatrizCoeficientes[2][1]=SumatoriaXiCubo;   MatrizCoeficientes[2][2]=SumatoriaXiCuadYi;
    DeterminanteZ = DeterminanteMatrices(MatrizCoeficientes);

    CoefXCuad = ((DeterminanteX)/(Determinante));
    CoefX = ((DeterminanteY)/(Determinante));
    Independiente = ((DeterminanteZ)/(Determinante));
}
