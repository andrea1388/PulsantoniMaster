#include <Arduino.h>

class RxPkt {
    public:
        byte *dati;
        byte destinatario;
        byte mittente;
        unsigned long int micros;
        byte indirizzobest[5];
        byte segnalebest[5];
        byte tipo;
        byte batteria;
        RxPkt(char *,byte);
};