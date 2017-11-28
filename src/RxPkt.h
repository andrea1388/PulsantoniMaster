#include <Arduino.h>

class RxPkt {
    public:
        byte *dati;
        byte destinatario;
        byte mittente;
        unsigned long int micros;
        byte indirizzobest[5];
        char segnalebest[5];
        byte tipo;
        byte batteria;
        RxPkt(byte *,byte);
};