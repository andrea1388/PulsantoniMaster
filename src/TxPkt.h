#include <Arduino.h>
class TxPkt {
    public:
        byte dest;
        byte rip;
        bool modovoto;
        byte *dati;
        byte len;
        TxPkt();
};