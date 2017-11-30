#include <Arduino.h>
#include "TxPkt.h"
TxPkt::TxPkt(byte mittente, byte destinatario,bool modovoto) {
    dati[0]=mittente;
    dati[1]=destinatario;
    dati[2]=(modovoto == true) ? 0xaa : 0x55;
    len=3;
}