#include <Arduino.h>
class TxPkt {
    public:
        TxPkt(byte destinatario, byte via, bool modovoto);
        byte *dati;
        byte len;

    protected:
        byte dest;
        byte rip;
        bool modovoto;

};
    