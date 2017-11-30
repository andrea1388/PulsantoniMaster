#include <Arduino.h>
class TxPkt {
    public:
        TxPkt(byte mittente, byte destinatario,bool modovoto);
        byte mittente;
        byte dati[3];
        byte len;

};
    