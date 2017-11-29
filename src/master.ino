#include <RFM69.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <Cmd.h>
#include "Antirimbalzo.h"
#include "TxPkt.h"
#include "RxPkt.h"
#include <SPI.h>



// parametri radio
#define NETWORKID 27
#define FREQUENCY 868000000
#define RFM69_CS 53
#define RFM69_IRQ 21
#define RFM69_IRQN 2 
#define RFM69_RST 49

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






#define LEDVERDE 41
#define LEDROSSO 43
#define LEDBLU   45
#define pinPULSANTE 47
#define numBestSlave 5 // numero di best che lo slave manda al master




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
    bool votato;
    bool sincronizzato;
    byte indirizzoRipetitoreCorrente;
    byte fallimenti;
    byte batteria;
    Slave(byte);
    ~Slave();
};
Slave::Slave(byte ind) {
  indirizzoRipetitoreCorrente=0; // trasmetti direttamente
  nodoripetitore=false;
  fallimenti=0;
  segnale=-127;
  indirizzo=ind;
  oravoto=0;
  for(byte i=0;i<5;i++)
  {
    best[i].segnale=-127;
    best[i].indirizzo=0;
  }
}
Slave::~Slave() {
}

typedef enum {ZERO, POLLING, VOTO, FINEVOTO} valoristato;

// variabili globali
valoristato stato;
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
RFM69 radio(RFM69_CS,RFM69_IRQ,true,RFM69_IRQN);
byte numero_max_slave;
byte ic; // indirizzo slave corrente
unsigned long t_inizio_voto;
unsigned int ttrapolls; // millisecondi tra un poll e quello dello slave successivo
bool timeoutNodoCorrente; // indica se il poll allo slave corrente fallisce
byte maxFallimenti; // numero max di fallimenti oltre il quale è dichiarato morto
byte numeroSalti; // numero di volte che il poll viene saltato quando è dichiarato morto
Slave** slave;
Slave* best[5]; // gli slave con votazioni migliori
byte numero_votati,indirizzo_slave_discovery;
byte numero_slave;
Antirimbalzo swVoto;
unsigned int intervallopollnormale;
unsigned int intervallopollvoto;
unsigned long int ttx; // istante della trasmissione (micros)
bool stampainfopoll=false;
bool stampainfosceltarip=false;
bool stampadatiradio=false;

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

void serialCmdStampaInfoRouting(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  if(args[1][0]=='0') stampainfosceltarip=false;
  if(args[1][0]=='1') stampainfosceltarip=true;
}

void serialCmdStampaInfoPoll(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  if(args[1][0]=='0') stampainfopoll=false;
  if(args[1][0]=='1') stampainfopoll=true;
}

void serialCmdStampaDatiRadio(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  if(args[1][0]=='0') stampadatiradio=false;
  if(args[1][0]=='1') stampadatiradio=true;
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
  radio._printpackets=false;
  swVoto.cbInizioStatoOn=PulsanteClickCorto;
  ic=1;
  ttrapolls=1000;
  maxFallimenti=10;
  numeroSalti=10;
  intervallopollnormale=1000;
  intervallopollvoto=1;
  stato=ZERO;

  cmdInit(&Serial);
  cmdAdd("Z", serialCmdInizioVoto);
  cmdAdd("Q", serialCmdFineVoto);
  cmdAdd("S", serialCmdMemorizzaNumSlave);
  cmdAdd("N", serialCmdLeggiNumSlave);
  cmdAdd("SP", serialCmdStampaPacchettiRadio);
  cmdAdd("IR", serialCmdStampaInfoRouting);
  cmdAdd("IP", serialCmdStampaInfoPoll);
  cmdAdd("DR", serialCmdStampaDatiRadio);
}
void CreaListaSlave(byte numslave) {
  if (slave) {
    for (int j=1;j<numero_max_slave+1;j++) {
      slave[j]->~Slave();
    }
    free(slave);   
  }
  slave=(Slave **)malloc(sizeof(Slave*)*numslave+2);
  for (int j=1;j<numslave+2;j++) {
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
      if(modoVoto && !slave[ic]->sincronizzato) salta=true;
      if(stampainfopoll & salta) 
      {
        Serial.print("salto");
        Serial.print("\t");
        Serial.print(ic);
        Serial.print("\t");
        Serial.print(slave[ic]->fallimenti);
        Serial.print("\t");
        Serial.println(slave[ic]->sincronizzato);
      }

    } while (salta);
    InterrogaNodoCorrente();
    timeoutNodoCorrente=true; // viene messo a false se vengono ricevuti i dati

  }

}

void ElaboraTimeOutNodoCorrente() {
  slave[ic]->fallimenti++;
  if(slave[ic]->fallimenti>maxFallimenti)
  {
    slave[ic]->fallimenti=maxFallimenti+numeroSalti;
    slave[ic]->sincronizzato=false;
  } 
  
}

void InterrogaNodoCorrente() {
  TrovaMigliorRipetitorePerNodo(ic);
  TxPkt p(1,ic,modoVoto);
  byte rip=(slave[ic]->indirizzoRipetitoreCorrente==1) ? ic : slave[ic]->indirizzoRipetitoreCorrente;
  radio.send(rip,p.dati,p.len,false);
  ttx=micros();
    if(stampainfopoll) 
    {
      Serial.print("poll");
      Serial.print("\t");
      Serial.print(ic);
      Serial.print("\t");
      Serial.print(rip);
      Serial.print("\t");
      Serial.println(ttx);
    }
}

void TrovaMigliorRipetitorePerNodo(byte dest) {
  // se l'ultima trasmissione è andata bene mantiene l'ultimo ripetitore
  if (slave[dest]->fallimenti==0) return;
  int pmax=0; 
  slave[dest]->indirizzoRipetitoreCorrente=1;
  for(byte j=0;j<numBestSlave;j++)
  {
    if(slave[dest]->best[j].indirizzo!=0) {
      if(slave[slave[dest]->best[j].indirizzo]->fallimenti==0) 
      {
        if(slave[dest]->best[j].indirizzo!=slave[dest]->indirizzoRipetitoreCorrente)
        {
          int p=(128 + slave[dest]->best[j].segnale) * (128 + slave[slave[dest]->best[j].indirizzo]->segnale);
          if(stampainfosceltarip)
          {
            Serial.print("sceltarip");
            Serial.print("\t");
            Serial.print(slave[dest]->best[j].indirizzo);
            Serial.print("\t");
            Serial.print(slave[dest]->best[j].segnale);
            Serial.print("\t");
            Serial.print(slave[slave[dest]->best[j].indirizzo]->segnale);
            Serial.print("\t");
            Serial.print(p);
            Serial.print("\t");
            Serial.println(ttx);
          }
          if (p>pmax) 
          {
            pmax=p;
            slave[dest]->indirizzoRipetitoreCorrente=slave[dest]->best[j].indirizzo;
          }
        }
      }
    }
  }
}


void ElaboraRadio() {
  if(radio.receiveDone()) {
    slave[radio.SENDERID]->segnale=radio.RSSI;
    RxPkt msg((byte *)radio.DATA,radio.DATALEN);
    if (stampadatiradio)
    {
        Serial.print("radio");
        Serial.print("\t");
        Serial.print(radio.SENDERID);
        Serial.print("\t");
        Serial.print(radio.TARGETID);
        Serial.print("\t");
        Serial.print(radio.RSSI);
        Serial.print("\t");
        Serial.print(msg.mittente);
        Serial.print("\t");
        Serial.print(msg.destinatario);
        Serial.print("\t");
        Serial.println(msg.tipo);
    }
    if(msg.destinatario==1) {
      if(msg.mittente==ic) {
        timeoutNodoCorrente=false;
        slave[ic]->fallimenti=0;
        switch(msg.tipo)
        {
          case 1:
            slave[ic]->deltat=msg.micros-ttx;
            slave[ic]->batteria=msg.batteria;
            slave[ic]->sincronizzato=true;
            if (stampadatiradio)
            {
              Serial.print("pkt1");
              Serial.print("\t");
              Serial.print(msg.micros);
              Serial.print("\t");
              Serial.print(msg.batteria);
              Serial.print("\t");
              Serial.println(slave[ic]->deltat);
            }
            break;
          case 2:
            if (stampadatiradio)
            {
              Serial.print("pkt2");
              Serial.print("\t");
            }
            for(byte j=0;j<5;j++)
            {
              slave[ic]->best[j].indirizzo=msg.indirizzobest[j];
              slave[ic]->best[j].segnale=msg.segnalebest[j];
              if (stampadatiradio)
              {
                Serial.print(msg.indirizzobest[j]);
                Serial.print("\t");
                Serial.println(msg.segnalebest[j]);
              }
            }
            break;
          case 3:
            slave[ic]->oravoto=msg.micros-slave[ic]->deltat;
            slave[ic]->votato=true;
            if(modoVoto && !slave[ic]->votato) NuovoVotoRicevutoDaSlave(msg.mittente);
            if (stampadatiradio)
            {
              Serial.print("pkt3");
              Serial.print("\t");
              Serial.println(msg.micros);
              Serial.print("\t");
              Serial.println(slave[ic]->oravoto);
            }
            break;
          case 4:
            slave[ic]->votato=false;
            if (stampadatiradio)
            {
              Serial.println("pkt4");
            }
            break;
        }
      }
    }
  }
}




void PulsanteClickCorto() {
  switch(stato)
  {
    case ZERO:
      stato=POLLING;
      break;
    case POLLING:
      stato=VOTO;
      InizioVoto();
      break;
    case VOTO:
      stato=FINEVOTO;
      FineVoto();
      break;
    case FINEVOTO:
      stato=ZERO;
      break;
  }
}

void InizioVoto() {
  bool ok=true;
  byte j;
  for(j=1;j<numero_max_slave;j++) 
  {
    slave[j]->votato=false;
  }
  ttrapolls=intervallopollvoto;
  for(j=0;j<5;j++) 
  {
    best[j]=0;
  }
}

void FineVoto() {
  ttrapolls=intervallopollnormale;
}

void NuovoVotoRicevutoDaSlave(byte i) {
  for(int y=0;y<5;y++) {
    if(best[y]==0) 
    {
      best[y]=slave[i];
      MostraRisultatiVoto();
      break;
    }
    else {
      if(slave[i]->oravoto<best[y]->oravoto) 
      {
        for(int f=4;f>y;f--)
              best[f]=best[f-1];
        best[y]=slave[i];
        MostraRisultatiVoto();
        break;
      }
    }
  }

}


void AggiornaDisplayKo() {
  int x=tft.getCursorX();
  int y=tft.getCursorY();
  tft.fillRect(0,200,320,40,COLORESFONDO);
  tft.setTextColor(RED);
  tft.setCursor(0, 205);
  for (int i=0;i<numero_slave;i++) {
    if(!slave[i]->sincronizzato) {
      tft.print(i);
      tft.print(" ");
    }
  }
  tft.setTextColor(COLORETESTONORMALE);
  tft.setCursor(x, y);  
}

void MostraStatoSlave() 
{
  tft.fillScreen(COLORESFONDO);
  tft.setCursor(0, 0);
  tft.println(F("Stato Slave"));
  for (int i=0;i<numero_slave;i++)
  {
    if(!slave[i]->sincronizzato)
    {
      tft.setTextColor(COLORETESTONORMALE);
      tft.print(indirizzo_slave_discovery);
      tft.print(" ");
    }
    else
    {
      tft.setTextColor(RED);
      tft.print(indirizzo_slave_discovery);
      tft.print(" ");
    }
  }  
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



