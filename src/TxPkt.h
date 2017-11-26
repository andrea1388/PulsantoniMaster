#include <Arduino.h>
class TxPkt {
    public:
        TxPkt(byte destinatario,bool modovoto);
        byte *dati;
        byte len;

};
    