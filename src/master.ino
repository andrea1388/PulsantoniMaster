#include <RFM69.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <Cmd.h>
#include "Antirimbalzo.h"
#include "TxPkt.h"
#include "RxPkt.h"

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
    unsigned long deltat;
    char segnale;
    Nodo best[5];
    bool nodoripetitore;
    byte indirizzoRipetitoreCorrente;
    byte fallimenti;
    Slave(byte);
    ~Slave();
};
Slave::Slave(byte ind) {
  indirizzoRipetitoreCorrente=0;
  nodoripetitore=false;
  fallimenti=0;
  segnale=-127;
  indirizzo=ind;
}
Slave::~Slave() {

}



RFM69 radio(RFM69_CS,RFM69_IRQ,true,RFM69_IRQN);
byte numero_max_slave;
byte ic; // indirizzo slave corrente
unsigned long t_inizio_voto;
unsigned int ttrapolls; // millisecondi tra un poll e quello dello slave successivo
#define MAXBESTNEIGHBOURS 5
Nodo* bestn[MAXBESTNEIGHBOURS];
bool timeoutNodoCorrente; // indica se il poll allo slave corrente fallisce
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
byte maxFallimenti; // numero max di fallimenti oltre il quale è dichiarato morto
#define numBestSlave 5 // numero di best che lo slave manda al master
byte numeroSalti; // numero di volte che il poll viene saltato quando è dichiarato morto
Slave** slave;
Slave* best[5];
byte numero_votati,indirizzo_slave_discovery;
byte numero_slave;
Antirimbalzo swVoto;
bool modoVoto;
unsigned int intervallopollnormale;
unsigned int intervallopollvoto;

// eventi comandi seriali
void serialCmdInizioVoto(int arg_cnt, char **args)
{
    InizioVoto();
}
void serialCmdFineVoto(int arg_cnt, char **args)
{
    FineVoto();
}

void serialCmdMemorizzaNumSlave(int arg_cnt, char **args)
{
    byte nums=atoi((const char *)args[1]);
    if(nums<1 || nums>254) {
      Serial.println(F("e parametro errato"));  
    } else {
      EEPROM.write(1,nums);
      CreaListaSlave(nums);
      Serial.print(F("e numero memorizzato: "));
      Serial.println(nums);
    }
}

void serialCmdLeggiNumSlave(int arg_cnt, char **args)
{
   Serial.print(F("ns "));
   Serial.println(numero_max_slave);
}

void serialCmdStampaPacchettiRadio(int arg_cnt, char **args)
{
  radio._printpackets=!radio._printpackets;
  Serial.print(F("stampapacchetti: "));
  Serial.println(radio._printpackets);  
}

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
  CreaListaSlave(EEPROM.read(1));
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
  //radio.readAllRegs();
  tft.print(F("Frequenza: "));
  tft.println(radio.getFrequency());
  tft.println();
  tft.println(F("Click lungo: Discovery"));
  tft.println(F("Click corto: Voto"));
  tft.println(F("Effettuare il Discovery"));
  tft.println();
  for (int i=0;i<MAXBESTNEIGHBOURS;i++) bestn[i]=new Nodo();
  radio._printpackets=false;
  swVoto.cbInizioStatoOn=PulsanteClickCorto;
  ic=1;
  ttrapolls=1000;
  maxFallimenti=10;
  numeroSalti=10;
  intervallopollnormale=1000;
  intervallopollvoto=1;

  cmdInit(&Serial);
  cmdAdd("Z", serialCmdInizioVoto);
  cmdAdd("Q", serialCmdFineVoto);
  cmdAdd("S", serialCmdMemorizzaNumSlave);
  cmdAdd("N", serialCmdLeggiNumSlave);
  cmdAdd("P", serialCmdStampaPacchettiRadio);


}
void CreaListaSlave(byte numslave) {
  if (slave) {
    for (int j=1;j<numero_max_slave+1;j++) {
      slave[j]->~Slave();
    }
    free(slave);   
  }
  slave=(Slave **)malloc(sizeof(Slave*)*numslave+1);
  for (int j=1;j<numslave+1;j++) {
    slave[j]=new Slave(j);
  }
  slave[1]->segnale=0; // 0 dBm = fisso forte
  numero_max_slave=numslave;

}
void loop() {
  swVoto.Elabora(digitalRead(pinPULSANTE)==LOW);
  Poll();
  ElaboraRadio();
}

void Poll() {
  static unsigned long tlastpoll;
  if((millis() - tlastpoll) > ttrapolls) {
    tlastpoll=millis();
    if (timeoutNodoCorrente) ElaboraTimeOutNodoCorrente();
    // aumenta indirizzo_corrente
    bool salta;
    byte iterazioni=0;
    do {
      salta=false;
      iterazioni++;
      ic++;
      if(ic>numero_max_slave+1) ic=2;
      //if(slave[ic]->ripetitore) salta=true;
      if(slave[ic]->fallimenti>maxFallimenti) {slave[ic]->fallimenti--; salta=true;}
      if(iterazioni>numero_max_slave) return;
    } while (salta);
    InterrogaNodoCorrente();
    timeoutNodoCorrente=true; // viene messo a false se vengono ricevuti i dati

  }

}

void ElaboraTimeOutNodoCorrente() {
  slave[ic]->fallimenti++;
  if(slave[ic]->fallimenti>maxFallimenti) slave[ic]->fallimenti=maxFallimenti+numeroSalti;
}

void InterrogaNodoCorrente() {
  TxPkt p;
  p.dest=ic;
  p.rip=TrovaMigliorRipetitorePerNodo(ic);
  p.modovoto=modoVoto;
  Trasmetti(&p);
}

byte TrovaMigliorRipetitorePerNodo(byte dest) {

  if (slave[dest]->fallimenti==0 && slave[dest]->indirizzoRipetitoreCorrente!=0) return slave[dest]->indirizzoRipetitoreCorrente;
  byte newrip=0;
  do {
    int pmax=0; 
    for(byte j=0;j<numBestSlave;j++)
    {
      int p=(128 + slave[dest]->best[j].segnale) * (128 + slave[slave[dest]->best[j].indirizzo]->segnale);
      if (p>pmax) {pmax=p;newrip=slave[dest]->best[j].indirizzo;}
    }

  } while (newrip!=slave[dest]->indirizzoRipetitoreCorrente);
  slave[dest]->indirizzoRipetitoreCorrente=newrip;
  return newrip;

}

void Trasmetti(TxPkt *p) {
  radio.send(p->rip,p->dati,p->len,false);
}



void ElaboraRadio() {
  if(radio.receiveDone()) {
    slave[radio.SENDERID]->segnale=radio.RSSI;
    RxPkt msg(radio.DATA,radio.DATALEN);
    if(msg.destinatario==1) {
      if(msg.mittente==ic) {
        timeoutNodoCorrente=false;
        slave[ic]->deltat=msg.micros-micros();
        if(modoVoto) NuovoVotoRicevutoDaSlave(msg.mittente);
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
    slave[j]->oravoto=0;
  ttrapolls=intervallopollvoto;
}

void FineVoto() {
  ttrapolls=intervallopollnormale;
}

void NuovoVotoRicevutoDaSlave(byte ind) {

}


void AggiornaDisplayKo() {
  int x=tft.getCursorX();
  int y=tft.getCursorY();
  tft.fillRect(0,200,320,40,COLORESFONDO);
  tft.setTextColor(RED);
  tft.setCursor(0, 205);
  for (int i=0;i<numero_slave;i++) {
    if(slave[i]->fallimenti>maxFallimenti) {
      tft.print(i);
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

