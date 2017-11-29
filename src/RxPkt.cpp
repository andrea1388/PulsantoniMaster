#include <Arduino.h>
#include "RxPkt.h"

RxPkt::RxPkt(byte *dati,byte len) {
    unsigned long int tmp;
    if(len<3) return;
    mittente=dati[0];
    destinatario=dati[1];
    tipo=dati[2];
    switch(tipo)
    {
        case 1:
            tmp=dati[3];
            tmp=tmp<<24;
            micros=tmp;
            tmp=dati[4];
            tmp=tmp<<16;
            micros+=tmp;
            tmp=dati[5];
            tmp=tmp<<8;
            micros+=tmp;
            micros+=dati[6]; 
            batteria=dati[7];
            break;
        case 2:
            for(int j=0;j<5;j++)
            {
                indirizzobest[j]=dati[2*j+3];
                segnalebest[j]=dati[2*j];
            }
            break;
        case 3:
            tmp=dati[3];
            tmp=tmp<<24;
            micros=tmp;
            tmp=dati[4];
            tmp=tmp<<16;
            micros+=tmp;
            tmp=dati[5];
            tmp=tmp<<8;
            micros+=tmp;
            micros+=dati[6]; 
            break;
        case 4:
            break;
    }

}