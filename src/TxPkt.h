#include <Arduino.h>
class TxPkt {
    public:
        TxPkt(byte mittente, byte destinatario,bool modovoto);
        byte mittente;
        byte *dati;
        byte len;

};
    