#include <Arduino.h>
#include "RxPkt.h"

RxPkt::RxPkt(byte *dati,byte len) {
    if(len<3) return;
    mittente=dati[0];
    destinatario=dati[1];
    tipo=dati[2];
    switch(tipo)
    {
        case 1:
            micros=*((unsigned long int*)dati[3]);
            batteria=dati[7];
            break;
        case 2:
            for(int j=0;j<5;j++)
            {
                indirizzobest[j]=dati[3+j];
                segnalebest[j]=dati[4+j];
            }
            break;
        case 3:
            micros=*((unsigned long int*)dati[3]);
            break;
        case 4:
            micros=*((unsigned long int*)dati[4]);
            break;
    }

}