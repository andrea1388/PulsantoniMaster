#include <Arduino.h>
#include "TxPkt.h"
TxPkt::TxPkt(byte destinatario,bool modovoto) {
    dati=new byte[2];
    dati[0]=destinatario;
    dati[1]=(modovoto == true) ? 0xaa : 0x55;
    len=2;
}