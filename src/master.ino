#include <RFM69.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include "Antirimbalzo.h"
#define DURATACLICKLUNGO 2000 // tempo pressione pulsante per click lungo = 2 secondi
#define TBACKOUTPULSANTE 100 // tempo blackout pulsante dopo un click
//stati
#define ZERO 0 // stato iniziale
#define DISCOVERY 1 // discovery: cerca gli slave presenti in rete
#define INVIASYNC 2 // pre voto. Invia l'ora del master a tutti gli slave svovati con discovery
#define VOTO 3 // fase di acquisizione dei voti e display risultati
// stati slave
#define FUORISYNC 1
#define VOTATO 2
#define NONVOTATO 3

// parametri radio
#define NETWORKID 27
#define FREQUENCY 868000000
#define RFM69_CS 53
#define RFM69_IRQ 21
#define RFM69_IRQN 2 
#define RFM69_RST 49
// stati per elaborazione seriale
#define COMANDO 0
#define VALORE 1
// dati display
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define COLORESFONDO BLACK
#define COLORETESTONORMALE GREEN



Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);


#define LEDVERDE 41
#define LEDROSSO 43
#define LEDBLU   45
#define pinPULSANTE 47
/*
class Display {
	public:
		Display(rs, enable, d4, d5, d6, d7);
		Print(char *);
		
}
Display::Print(char* s) {
	
}
*/

byte numero_votati,indirizzo_slave_discovery;
byte numero_slave;

class Nodo {
  public:
    byte indirizzo;
    char segnale;
    Nodo();
};
Nodo::Nodo() { indirizzo=0; segnale=-127;}

class Slave {
  public:
    byte indirizzo;
    unsigned long oravoto;
    char segnale;
    Nodo best[5];
    bool ripetitore;
    Slave(byte);
};
Slave::Slave(byte ind) {
  indirizzo=ind;
  segnale=-127;
  ripetitore=false;
}

Nodo** nodo;



class Stato {
  public:
    Stato();
    byte getStato();
    void setStato(byte);
  private:
    byte stato;
};

RFM69 radio(RFM69_CS,RFM69_IRQ,true,RFM69_IRQN);
byte numero_max_slave;
byte ic; // indirizzo slave corrente
unsigned long t_inizio_voto;
#define MAXBESTNEIGHBOURS 5
Nodo* bestn[MAXBESTNEIGHBOURS];
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

Antirimbalzo swVoto;

void setup() {
  pinMode(pinPULSANTE, INPUT_PULLUP);
  pinMode(LEDVERDE, OUTPUT);
  pinMode(LEDROSSO, OUTPUT);
  pinMode(LEDBLU, OUTPUT);
  digitalWrite(LEDVERDE, LOW);
  digitalWrite(LEDROSSO, HIGH);
  digitalWrite(LEDBLU, LOW);
  
  Serial.begin(9600);
  Serial.print(F("ns "));
  //lcd.begin(16, 2);
  numero_max_slave=EEPROM.read(1);
  Serial.println(numero_max_slave);

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
  tft.reset();
  uint16_t identifier = tft.readID();
  identifier=0x9341;
  tft.begin(identifier);
  tft.fillScreen(COLORESFONDO);
  tft.setCursor(0, 0);
  tft.setTextColor(RED);  
  tft.setTextSize(3);
  tft.setRotation(1);
  tft.println("Pulsantoni");
  tft.setTextSize(2);
  tft.setTextColor(COLORETESTONORMALE);  
  tft.println();
  tft.print("numero slave: ");
  tft.println(numero_max_slave);

  delay(20);
  radioSetup();
  nodo=(Nodo **)malloc(sizeof(&Nodo)*numero_max_slave+1);
  for (int j=0;j<numero_max_slave+1;j++) {
    nodo[j]=new Nodo(j+1);
  }
  nodo[0].segnale=0; // 0 dBm = fisso forte
  //radio.readAllRegs();
  tft.print(F("Frequenza: "));
  tft.println(radio.getFrequency());
  tft.println();
  tft.println(F("Click lungo: Discovery"));
  tft.println(F("Click corto: Voto"));
  tft.println(F("Effettuare il Discovery"));
  tft.println();
  for (int i=0;i<MAXBESTNEIGHBOURS;i++) bestn[i]=new Nodo();
  stato.setStato(ZERO);
  radio._printpackets=false;
  swVoto.cbInizioStatoOn=PulsanteClickCorto;
  ic=1;

}
// algoritmo 1
void loop() {
  ElaboraSeriale();
  ElaboraPulsante();
  ElaboraStato();
  swVoto.Elabora(digitalRead(pinPULSANTE)==LOW);
  Poll();
  ElaboraRadio();
}

void Poll() {
  static unsigned long tlastpoll;
  if(millis() - tlastpoll > ttrapolls) {
    tlastpoll=millis();
    if (timeoutNodoCorrente) ElaboraTimeOutNodoCorrente();
    // aumenta indirizzo_corrente
    bool salta;
    byte iterazioni=0;
    do {
      salta=false;
      iterazioni++;
      ic++
      if(ic>numero_max_slave+1) ic=2;
      //if(nodo[ic].ripetitore) salta=true;
      if(nodo[ic].fallimenti>maxFallimenti) {nodo[ic].fallimenti--; salta=true;}
      if(iterazioni>numero_max_slave) return;
    } while (salta);
    InterrogaNodoCorrente();
    timeoutNodoCorrente=true; // viene messo a false se vengono ricevuti i dati

  }

}

void ElaboraTimeOutNodoCorrente() {
  nodo[ic].fallimenti++;
  if(nodo[ic].fallimenti>maxFallimenti) nodo[ic].fallimenti=maxFallimenti+numeroSalti;
}

void InterrogaNodoCorrente() {
  byte rip=TrovaMigliorRipetitorePerNodo(ic);
  Pacchetto p;
  p.dest=ic;
  p.rip=TrovaMigliorRipetitorePerNodo(ic);
  p.modovoto=modovoto;
  Trasmetti(p);
}

byte TrovaMigliorRipetitorePerNodo(byte dest) {

  if (nodo[dest].fallimenti==0 && nodo[dest].ripetitore!=0) return nodo[dest].ripetitore;
  byte newrip=0;
  do {
    int pmax=0; 
    for(int j=0;j<numBestSlave;j++)
    {
      int p=(128 + nodo[dest].best[j].segnale) * (128 + nodo[nodo[dest].best[j].indirizzo].segnale);
      if (p>pmax) {pmax=p;newrip=nodo[dest].best[j].indirizzo;}
    }

  } while (newrip!=nodo[dest].ripetitore);
  nodo[dest].ripetitore=newrip;
  return newrip;

}
void CostruisciListaNodi(byte ind, int sign, byte len) {
    // se il nodo ricevuto è più forte del più debole lo sostituisco con questo
    if(ind<0 || ind >slave[numero_slave-1]->indirizzo) return;
    if(len<2) return;
    bool giainlista=false;
    for (int i=0;i<MAXBESTNEIGHBOURS;i++) if(bestn[i]->indirizzo==ind) {bestn[i]->segnale=sign; giainlista=true;}
    if(!giainlista) {
      int minimo=bestn[0]->segnale;
      byte indicemin=0;
      for (int i=0;i<MAXBESTNEIGHBOURS;i++) if(bestn[i]->segnale<minimo) {minimo=bestn[i]->segnale; indicemin=i;};
      if(sign>minimo) {bestn[indicemin]->segnale=sign; bestn[indicemin]->indirizzo=ind;}
    }
    Nodo *tmp;
    for (int i=0;i<MAXBESTNEIGHBOURS-1;i++) 
      for (int k=i+1;k<MAXBESTNEIGHBOURS;k++) 
        if(bestn[i]->segnale<bestn[k]->segnale) {tmp=bestn[k]; bestn[k]=bestn[i]; bestn[i]=tmp;}
    
    if(radio._printpackets) {
      Serial.print(F("best: i/s "));
      for (int i=0;i<MAXBESTNEIGHBOURS;i++) {
        Serial.print(bestn[i]->indirizzo);
        Serial.print("/");
        Serial.print(bestn[i]->segnale);
        Serial.print(" ");
        
      }
      Serial.println(" ");
    }
}

void ElaboraSeriale() {
  static byte comando=0,prossimodato=0,k=0,valore[5];
  int nums;
  if(Serial.available()) {
    char c=Serial.read();
    /*
    Serial.print(F("datiser: char="));
    Serial.print(c,HEX);
    Serial.print(F(" cmd="));
    Serial.print(comando,HEX);
    Serial.print(F(" prox="));
    Serial.print(prossimodato,HEX);
    Serial.print(F(" k="));
    Serial.println(k,HEX);
    */
    if(c==' ') return;
    if(c=='\n') {
      // elabora il comando
      switch(comando) {
        case 'Y':
          // discovery
          if(stato.getStato()==0) {
            stato.setStato(DISCOVERY);
          }
          break;
        case 'X':
          // termina discovery
          if(stato.getStato()==DISCOVERY) {
            stato.setStato(ZERO);
          }
          break;
        case 'Z':
          // inizia voto
          if(stato.getStato()==ZERO) {
            stato.setStato(INVIASYNC);
          }
          break;
        case 'Q':
          // termina voto
          if(stato.getStato()==VOTO) {
            tft.println(F("Voto concluso"));
            stato.setStato(ZERO);
          }
          // invia discovery a 1
          break;
        case 'S':
          // scrivi indirizzo numero max slave sul byte 1 della eeprom
          nums=atoi((const char *)valore);
          if(nums<1 || nums>254) {
            Serial.println(F("e parametro errato"));  
          } else {
            EEPROM.write(1,nums);
            numero_max_slave=nums;
            slave=(Slave **)realloc(slave,sizeof(Slave)*nums);
            Serial.print(F("e numero memorizzato: "));
            Serial.println(nums);
          }
          break;
        case 'N':
          // stampa il num max di client
          Serial.print(F("ns "));
          Serial.println(numero_max_slave);
          break;
        case 'P':
          // stampa il num max di client
          radio._printpackets=!radio._printpackets;
          Serial.print(F("stampapacchetti: "));
          Serial.println(radio._printpackets);
          break;
      }
      k=0;
      prossimodato=COMANDO;   
      return;  

    }
    if(prossimodato==COMANDO) {
      c=toupper(c);
      comando=c;
      prossimodato=VALORE;
      k=0;
      return;
    }
    if(prossimodato==VALORE) {
      valore[k++]=c;
      valore[k] = 0;
      if(k>3) {
        k=0;
        prossimodato=COMANDO;  
        return;   
      }
    }
    
  }

}


//algoritmo 4
void ElaboraStato() {
  switch(stato.getStato()) {
    case DISCOVERY:
      digitalWrite(LEDVERDE, LOW);
      digitalWrite(LEDROSSO, LOW);
      digitalWrite(LEDBLU, HIGH);
      Discovery();
      break;
    case VOTO:
      digitalWrite(LEDVERDE, HIGH);
      digitalWrite(LEDROSSO, LOW);
      digitalWrite(LEDBLU, LOW);      
      Voto();
      break;
    case INVIASYNC:
      if(inviaSync()) stato.setStato(VOTO); else stato.setStato(ZERO);
      break;
  }
}


void Voto() {
  if(numero_votati==numero_slave) {
    tft.println(F("Voto concluso"));
      digitalWrite(LEDVERDE, LOW);
      digitalWrite(LEDROSSO, HIGH);
      digitalWrite(LEDBLU, LOW);
    stato.setStato(ZERO);    
  }
  else {
    interrogaTuttiGliSlave();
  }
}

void ElaboraRadio() {
  if(radio.receiveDone()) {
    nodo[radio.SENDERID].segnale=radio.RSSI;
    RxPkt pkt(radio.DATA);
    if(msg.dest==1) {
      if(msg.sender==ic) {
        timeoutNodoCorrente=false;
        nodo[pkt.mittente].deltat=msg.micros-micros();
        if(modovoto) NuovoVotoRicevutoDaSlave(pkt.mittente);
      }
    }
  }
}




void PulsanteClickCorto() {
  modoVoto=!modoVoto;
  if(modoVoto) InizioVoto(); else FineVoto();
}

void InizioVoto() {
  // cancella tutte le ore voto
  for(int j=1;j<numero_max_slave;j++)
    nodo[j].oravoto=0;
    intervallopoll=intervallopollvoto;
}

void FineVoto() {
  intervallopoll=intervallopollnormale;
}


void AggiornaDisplayKo() {
  int x=tft.getCursorX();
  int y=tft.getCursorY();
  tft.fillRect(0,200,320,40,COLORESFONDO);
  tft.setTextColor(RED);
  tft.setCursor(0, 205);
  for (int i=0;i<numero_slave;i++) {
    if(!slave[i]->funzionante) {
      tft.print(slave[i]->indirizzo);
      tft.print(" ");
    }
  }
  tft.setTextColor(COLORETESTONORMALE);
  tft.setCursor(x, y);  
}




void radioSetup() {
  // Hard Reset the RFM module 
  Serial.println("e radiosetup");
  pinMode(RFM69_RST, OUTPUT); 
  digitalWrite(RFM69_RST, HIGH); 
  delay(100);
  digitalWrite(RFM69_RST, LOW); 
  delay(100);
  radio.initialize(RF69_868MHZ,0,NETWORKID);
  /*
  radio.writeReg(0x03,0x0D); // 9k6
  radio.writeReg(0x04,0x05);
  */
  radio.writeReg(0x03,0x00); // 153k6
  radio.writeReg(0x04,0xD0);
  radio.writeReg(0x37,radio.readReg(0x37) | 0b01010000); // data whitening e no address filter
  radio.setFrequency(FREQUENCY);
  radio.setHighPower(); 
  radio.setPowerLevel(31);
  radio.promiscuous(true);
}




//algoritmo 11
// chiamato ad ogni giro di poll
void MostraRisultatiVoto() {
  tft.fillScreen(COLORESFONDO);
  tft.setCursor(0, 0);
  tft.setTextSize(3);
  tft.setTextColor(COLORETESTONORMALE);
  tft.println(F("Risultati voto:"));
  tft.println();
  for(int f=0;f<5;f++) {
    
    if(best[f]!=0) {
      /*
      Serial.print(f);
      Serial.print(" ");
      Serial.print(best[f]->indirizzo);
      Serial.print(" ");
      Serial.print(best[f]->oravoto);
      */
      if(f==0) {
          tft.setTextColor(RED);  
      }
      else {
          tft.setTextColor(CYAN);  
      }
      int ovi=best[f]->oravoto/1000000;
      int ovd=(best[f]->oravoto-ovi*1000000)/100;
      char str[25];
      sprintf(str, " %3d %3d,%04d",best[f]->indirizzo,ovi,ovd);
      /*
      tft.print(f+1);
      tft.print(": ");
      tft.print(best[f]->indirizzo);
      tft.print(": ");
      tft.print(ovi);
      tft.print(",");
      */
      tft.println(str);
    }
    /*
    else {
      Serial.print(f);
      Serial.print(" 0");

    }
    */
  }
  //Serial.println("");
  tft.setTextSize(2);
  tft.setTextColor(COLORETESTONORMALE);
  tft.println();
}

void stampapkt(byte *pkt,int len) {
  Serial.print("millis:");
  Serial.print(millis());
  Serial.print("len:");
  Serial.print(len);
  Serial.print("pkt:");
  for (int i=0;i<len;i++) {
    Serial.print(pkt[i],HEX);
    Serial.print(":");
  }
  Serial.println();
    
}

byte IndirizzoMigliorRipetitore(byte destinatario,byte tentativo) {
  if(tentativo==1) {
    if
  }

}