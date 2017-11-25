#include <Arduino.h>
#include "TxPkt.h"
TxPkt::TxPkt(byte destinatario, byte via,bool modovoto) {
    dati=new byte[3];
    dati[0]=destinatario;
    dati[1]=destinatario;
    dati[2]=(modovoto == true) ? 0xaa : 0x55;
    len=3;
}