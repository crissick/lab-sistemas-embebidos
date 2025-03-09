#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>
#include <conio.h>
#include <string.h>

unsigned int TiempoTotal, Horas_ms, Minutos_ms, Segundos_ms;
unsigned long int Clave = 0x10000000, Token;
unsigned char horaReal, minutoReal, segundoReal;
int i = 0;
unsigned char dato = 219;

void gotoxy(int x, int y) {
    COORD coord = {x, y};
    SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
}

void dibujarCuadro() {
    gotoxy(0, 11);
    for (i = 1; i <= 72; i++) {
        printf("%c", dato);
        if (i % 12 == 0)
            printf("\n");
    }
}

void animarBits() {
    int bit, bitActual;
    for (bit = 7; bit >= 0; bit--) {
        bitActual = (horaReal >> bit) & 1;
        system("cls");
        printf("mostrando bit (hora) -> Bit %d: %d (Valor: %u)\n\n\n\n\n\n\n\n\n\n\n", bit, bitActual, horaReal);
        for (i = 1; i <= 72; i++) {
            printf("%c", dato);
            if (i % 12 == 0)
                printf("\n");
        }
        if (bitActual == 1)
            Sleep(200);
        else
            Sleep(50);
        system("cls");
        for (i = 1; i <= 112; i++) {
            printf(" ");
            if (i % 12 == 0)
                printf("\n");
        }
        Sleep(50);
    }
    for (bit = 7; bit >= 0; bit--) {
        bitActual = (minutoReal >> bit) & 1;
        system("cls");
        printf("mostrando bit (minuto) -> Bit %d: %d (Valor: %u)\n\n\n\n\n\n\n\n\n\n\n", bit, bitActual, minutoReal);
        for (i = 1; i <= 72; i++) {
            printf("%c", dato);
            if (i % 12 == 0)
                printf("\n");
        }
        if (bitActual == 1)
            Sleep(200);
        else
            Sleep(50);
        system("cls");
        for (i = 1; i <= 112; i++) {
            printf(" ");
            if (i % 12 == 0)
                printf("\n");
        }
        Sleep(50);
    }
    for (bit = 7; bit >= 0; bit--) {
        bitActual = (segundoReal >> bit) & 1;
        system("cls");
        printf("mostrando bit (Segundo) -> Bit %d: %d (Valor: %u)\n\n\n\n\n\n\n\n\n\n\n", bit, bitActual, segundoReal);
        for (i = 1; i <= 72; i++) {
            printf("%c", dato);
            if (i % 12 == 0)
                printf("\n");
        }
        if (bitActual == 1)
            Sleep(200);
        else
            Sleep(50);
        system("cls");
        for (i = 1; i <= 112; i++) {
            printf(" ");
            if (i % 12 == 0)
                printf("\n");
        }
        Sleep(50);
    }
    system("cls");
    Sleep(90);
}

void actualizarTokenYTiempo(struct tm *tm_info) {
    // Si la hora es 0, se muestra como 24
    horaReal   = (tm_info->tm_hour == 0) ? 24 : tm_info->tm_hour;
    minutoReal = tm_info->tm_min;
    // Sumar 6 segundos de forma consistente:
    segundoReal = tm_info->tm_sec + 6;
    // Ahora usamos el segundo modificado en el cálculo:
    Horas_ms    = tm_info->tm_hour * 3600000;
    Minutos_ms  = tm_info->tm_min * 60000;
    // Usamos (tm_info->tm_sec + 6) para obtener el tiempo en ms.
    Segundos_ms = (tm_info->tm_sec + 6) * 1000;
    TiempoTotal = Horas_ms + Minutos_ms + Segundos_ms;
    Token = Clave ^ TiempoTotal;
}

int main() {
    char inputBuffer[100] = {0};
    int inputIndex = 0;
    char message[50] = "";
    time_t messageTime = 0;
    int lastTokenSec = -1;
    
    printf("Oprima Enter para comenzar...\n\n\n\n\n\n\n\n");
    printf("______X_____");
    getchar();
    
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    actualizarTokenYTiempo(tm_info);
    animarBits();
    
    system("cls");
    dibujarCuadro();
    
    while (1) {
        now = time(NULL);
        tm_info = localtime(&now);
        // Actualizamos el token solo cada 10 segundos.
        if (tm_info->tm_sec % 10 == 0 && tm_info->tm_sec != lastTokenSec) {
            actualizarTokenYTiempo(tm_info);
            lastTokenSec = tm_info->tm_sec;
        }
        
        gotoxy(0, 0);
        printf("TOKEN: %X        \n\n", Token);
        printf("Hora: %02u:%02u:%02u    \n\n", horaReal, minutoReal, segundoReal);
        if (message[0] != '\0') {
            if (difftime(now, messageTime) < 4)
                printf("%s  \n", message);
            else
                message[0] = '\0';
        } else {
            printf("                    \n");
        }
        gotoxy(0, 5);
        printf("Ingrese token: %s           ", inputBuffer);
        
        while (kbhit()) {
            char ch = getch();
            if (ch == '\r') {
                inputBuffer[inputIndex] = '\0';
                unsigned long int userToken = strtoul(inputBuffer, NULL, 16);
                if (userToken == Token)
                    strcpy(message, "Correcto");
                else
                    strcpy(message, "Incorrecto");
                messageTime = now;
                inputIndex = 0;
                inputBuffer[0] = '\0';
            } else if (ch == '\b') {
                if (inputIndex > 0) {
                    inputIndex--;
                    inputBuffer[inputIndex] = '\0';
                }
            } else {
                if (inputIndex < 99) {
                    inputBuffer[inputIndex++] = ch;
                    inputBuffer[inputIndex] = '\0';
                }
            }
        }
        Sleep(100);
    }
    
    return 0;
}

