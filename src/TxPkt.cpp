#include <Arduino.h>
#include "TxPkt.h"
TxPkt::TxPkt() {
    dati=new byte[1];
    dati[0]='P';
    len=1;
}