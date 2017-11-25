#include <Arduino.h>

class RxPkt {
    public:
        byte *dati;
        byte len;
        byte destinatario;
        byte mittente;
        unsigned long int micros;
        RxPkt(char *,byte);
};