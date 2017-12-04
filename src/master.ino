#include <RFM69.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <Cmd.h>
#include "Antirimbalzo.h"
#include "TxPkt.h"
#include "RxPkt.h"
#include <SPI.h>
#include <RFM69registers.h>



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
    long deltat;
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
  indirizzoRipetitoreCorrente=1; // trasmetti direttamente
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
Slave* slavemigliorvoto[5]; // gli slave con votazioni migliori
byte numero_votati,indirizzo_slave_discovery;
Antirimbalzo swVoto;
unsigned int intervallopollnormale;
unsigned int intervallopollvoto;
unsigned long int ttx; // istante della trasmissione (micros)
bool stampainfopoll=false;
bool stampainfosceltarip=false;
bool stampadatiradio=false;
bool displaypollingDaAggiornare;
byte numerocicli; // in modo voto concluso fa alcuni giri per comunicare la fine voto
float premiotrattadiretta=1.1; // premio per la tx diretta

// eventi comandi seriali
void serialCmdInizioVoto(int arg_cnt, char **args)
{
    InizioVoto();
}
void serialCmdFineVoto(int arg_cnt, char **args)
{
    FineVoto();
}
void serialCmdPrintRadioReg(int arg_cnt, char **args)
{
    radio.readAllRegs();
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
      ic=1;
    }
}
void serialCmdWriteRadioReg(int arg_cnt, char **args)
{
    if(arg_cnt!=3) return;
    byte reg=atoi((const char *)args[1]);
    byte val=atoi((const char *)args[2]);
    if(reg<0) return;
    if(reg>0x71) return;
    radio.writeReg(reg,val);
    Serial.print(reg,HEX);
    Serial.print(" ");
    Serial.println(val,HEX);
}

void serialCmdLeggiNumSlave(int arg_cnt, char **args)
{
   Serial.print(F("ns "));
   Serial.println(numero_max_slave);
}

void serialCmdStampaPacchettiRadio(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  if(args[1][0]=='0') radio._printpackets=false;
  if(args[1][0]=='1') radio._printpackets=true;
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
  
  Serial.begin(250000);
  Serial.print(F("ns "));
  //lcd.begin(16, 2);
  CreaListaSlave(EEPROM.read(1));
  Serial.println(numero_max_slave);

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
  tft.reset();
  uint16_t identifier = tft.readID();
  identifier=0x9341;
  tft.begin(identifier);

  delay(20);
  radioSetup();
  //radio.readAllRegs();
  radio._printpackets=false;
  swVoto.cbInizioStatoOn=PulsanteClickCorto;
  swVoto.tPeriodoBlackOut=50;
  ttrapolls=1000;
  maxFallimenti=10;
  numeroSalti=10;
  intervallopollnormale=1000;
  intervallopollvoto=100;
  stato=ZERO;
  StatoZero();

  cmdInit(&Serial);
  cmdAdd("z", serialCmdInizioVoto);
  cmdAdd("q", serialCmdFineVoto);
  cmdAdd("s", serialCmdMemorizzaNumSlave);
  cmdAdd("n", serialCmdLeggiNumSlave);
  cmdAdd("ip", serialCmdStampaPacchettiRadio);
  cmdAdd("ir", serialCmdStampaInfoRouting);
  cmdAdd("io", serialCmdStampaInfoPoll);
  cmdAdd("ia", serialCmdStampaDatiRadio);
  cmdAdd("radioreg", serialCmdPrintRadioReg);
  cmdAdd("writeradioreg", serialCmdWriteRadioReg);

  randomSeed(analogRead(0));
}
void CreaListaSlave(byte numslave) {
  if (slave) {
    for (int j=1;j<numero_max_slave+2;j++) {
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
  if(stato==POLLING || stato==VOTO || (stato==FINEVOTO && numerocicli>0))
  {
    Poll();
    ElaboraRadio();
  } 
  cmdPoll();
  if(stato==POLLING && displaypollingDaAggiornare) AggiornaDisplayPolling();
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
      if(ic>numero_max_slave+1) {ic=2; if(numerocicli>0) numerocicli--;}
      //if(slave[ic]->ripetitore) salta=true;
      if(slave[ic]->fallimenti>maxFallimenti) {if(random(300)>50) salta=true;};
      if(iterazioni>numero_max_slave) return;
      if(stato==VOTO && !slave[ic]->sincronizzato) salta=true;
      if(stampainfopoll & salta) 
      {
        Serial.print("salto");
        Serial.print("\t");
        Serial.print(ic);
        Serial.print("\t");
        Serial.println(slave[ic]->fallimenti);
      }

    } while (salta);
    InterrogaNodoCorrente();
    timeoutNodoCorrente=true; // viene messo a false se vengono ricevuti i dati
    //if (stato==POLLING) displaypollingDaAggiornare=true;

  }

}

void ElaboraTimeOutNodoCorrente() {
  slave[ic]->fallimenti++;
  if(slave[ic]->fallimenti==4)
  {
    displaypollingDaAggiornare=true;  
    Serial.print("J ");
    Serial.println(ic);
  } 
  if(slave[ic]->fallimenti>maxFallimenti)
  {
    slave[ic]->fallimenti=maxFallimenti+numeroSalti;
    slave[ic]->sincronizzato=false;
  } 
  
}

void InterrogaNodoCorrente() {
  TrovaMigliorRipetitorePerNodo(ic);
  TxPkt p(1,ic,stato==VOTO);
  byte rip=(slave[ic]->indirizzoRipetitoreCorrente==1) ? ic : slave[ic]->indirizzoRipetitoreCorrente;
  radio.send(rip,p.dati,p.len,false);
  ttx=micros();
  radio.receiveDone();
  if(stampainfopoll) 
    {
      Serial.print("poll");
      Serial.print("\t");
      Serial.print(slave[ic]->indirizzo);
      Serial.print("\t");
      Serial.print(rip);
      Serial.print("\t");
      Serial.print(slave[ic]->fallimenti);
      Serial.print("\t");
      Serial.print(slave[ic]->sincronizzato);
      Serial.print("\t");
      Serial.println(ttx);
    }
}

void TrovaMigliorRipetitorePerNodo(byte dest) {
  // se l'ultima trasmissione è andata bene mantiene l'ultimo ripetitore
  if (slave[dest]->fallimenti==0) return;
  int pmax=0; 
  slave[dest]->indirizzoRipetitoreCorrente=1;
  if(stampainfosceltarip)
  {
    Serial.print("sceltarip per ");
    Serial.print(dest);
    Serial.print("\t");
  }
  for(byte j=0;j<numBestSlave;j++)
  {
    if(slave[dest]->best[j].indirizzo!=0) {
      if(slave[slave[dest]->best[j].indirizzo]->fallimenti==0) 
      {
        if(slave[dest]->best[j].indirizzo!=slave[dest]->indirizzoRipetitoreCorrente)
        {
          int p=(128 + slave[dest]->best[j].segnale) * (128 + slave[slave[dest]->best[j].indirizzo]->segnale);
          if(slave[dest]->best[j].indirizzo==1) p=p*premiotrattadiretta;
          if(stampainfosceltarip)
          {
            Serial.print(slave[dest]->best[j].indirizzo);
            Serial.print("\t");
            Serial.print(slave[dest]->best[j].segnale,DEC);
            Serial.print("\t");
            Serial.print(slave[slave[dest]->best[j].indirizzo]->segnale,DEC);
            Serial.print("\t");
            Serial.print(p);
            Serial.print("\t");
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
  if(stampainfosceltarip)
  {
    Serial.print("scelto: ");
    Serial.println(slave[dest]->indirizzoRipetitoreCorrente);
  }
}


void ElaboraRadio() {
  if(radio.receiveDone()) {
    if (radio._printpackets) stampapkt(radio.SENDERID,radio.TARGETID,radio.DATA,radio.DATALEN);
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
        if(slave[ic]->fallimenti>0) 
        {
          displaypollingDaAggiornare=true;
          Serial.print("K ");
          Serial.println(ic);
        }
        slave[ic]->fallimenti=0;
        switch(msg.tipo)
        {
          case 1:
            // lo slave è in modo nonvoto e invia info sync e batteria
            slave[ic]->deltat=msg.micros-ttx;
            if(slave[ic]->batteria!=msg.batteria) 
            {
              Serial.print("M ");
              Serial.print(ic);
              Serial.print(" ");
              Serial.println(msg.batteria);
            }
            slave[ic]->batteria=msg.batteria;
            if(!slave[ic]->sincronizzato) 
            {
              displaypollingDaAggiornare=true;
              Serial.print("L ");
              Serial.println(ic);
            }
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
            // lo slave è in modo nonvoto e invia info routing
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
                Serial.println(msg.segnalebest[j],DEC);
              }
            }
            break;
          case 3:
            // lo slave è in modo voto e ha votato
            slave[ic]->oravoto=msg.micros-slave[ic]->deltat-t_inizio_voto;
            if(stato==VOTO && !slave[ic]->votato) {slave[ic]->votato=true; NuovoVotoRicevutoDaSlave(msg.mittente);}
            if (stampadatiradio)
            {
              Serial.print("pkt3");
              Serial.print("\t");
              Serial.print(msg.micros);
              Serial.print("\t");
              Serial.println(slave[ic]->oravoto);
            }
            break;
          case 4:
            // lo slave è in modo nonvoto e non ha votato
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
      PollingIniziato();
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
      stato=POLLING;
      PollingIniziato();
      break;
  }
}

// gestione Eventi 
void StatoZero()
{
  tft.fillScreen(COLORESFONDO);
  tft.setCursor(0, 0);
  tft.setTextColor(RED);  
  tft.setTextSize(3);
  tft.setRotation(1);
  tft.println("Pulsantoni");
  tft.setTextColor(COLORETESTONORMALE);  
  tft.setTextSize(2);
  tft.print(F("Frequenza: "));
  tft.println(radio.getFrequency());
  tft.print("numero slave: ");
  tft.println(numero_max_slave);
  tft.println(F("pressione pulsante"));
  tft.println(F("1) start polling"));
  tft.println(F("2) modo voto"));
  tft.println(F("3) fine voto"));
}

void InizioVoto() {
  byte j;
  for(j=2;j<numero_max_slave+2;j++) 
  {
    slave[j]->votato=false;
  }
  ttrapolls=intervallopollvoto;
  for(j=0;j<5;j++) 
  {
    slavemigliorvoto[j]=0;
  }
  tft.fillScreen(COLORESFONDO);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.print(F("Voto in corso:\n"));
  t_inizio_voto=micros();
  displaypollingDaAggiornare=false;
  Serial.println("H");
}

void PollingIniziato()
{
  tft.fillScreen(COLORESFONDO);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.print(F("Polling iniziato\n"));
  tft.setCursor(0, 9);
  tft.setTextSize(1);
  ttrapolls=intervallopollnormale;
  byte j;
  for(j=2;j<numero_max_slave+2;j++) 
  {
    slave[j]->fallimenti=0;
    slave[j]->sincronizzato=false;
  }
  displaypollingDaAggiornare=true;
  ic=1;
  Serial.println("G");
}

void FineVoto() {
  tft.fillRect(0,0,320,16,COLORESFONDO);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.print(F("Voto concluso:\n"));
  ttrapolls=intervallopollnormale;
  numerocicli=30;
  Serial.println("I");

}

void NuovoVotoRicevutoDaSlave(byte i) {
  for(int y=0;y<5;y++) {
    if(slavemigliorvoto[y]==0) 
    {
      slavemigliorvoto[y]=slave[i];
      MostraRisultatiVoto();
      break;
    }
    else {
      if(slave[i]->oravoto<slavemigliorvoto[y]->oravoto) 
      {
        for(int f=4;f>y;f--)
              slavemigliorvoto[f]=slavemigliorvoto[f-1];
        slavemigliorvoto[y]=slave[i];
        MostraRisultatiVoto();
        break;
      }
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
  radio.initialize(RF69_868MHZ,1,NETWORKID);
  radio.writeReg(REG_BITRATEMSB,RF_BITRATEMSB_50000);
  radio.writeReg(REG_BITRATELSB,RF_BITRATELSB_50000);
  radio.writeReg(REG_FDEVMSB,RF_FDEVMSB_50000);
  radio.writeReg(REG_FDEVLSB,RF_FDEVLSB_50000);
  radio.writeReg(REG_PACKETCONFIG1,RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF); // data whitening e no address filter
  radio.setFrequency(FREQUENCY);
  radio.setHighPower(); 
  radio.setPowerLevel(31);
  radio.promiscuous(true);
}

void AggiornaDisplayPolling() {
  tft.fillRect(0,17,320,223,COLORESFONDO);
  tft.setCursor(0, 17);
  tft.setTextSize(2);
  for (int i=2;i<numero_max_slave+2;i++) {
    if(slave[i]->fallimenti>3) {
      tft.setTextColor(RED);
    } 
    else
    {
      if(!slave[i]->sincronizzato) tft.setTextColor(GREEN); 
      else tft.setTextColor(BLUE);
    }
    if(slave[i]->nodoripetitore) tft.setTextColor(CYAN);
    tft.print(slave[i]->indirizzo);
    tft.print(" ");
  }
  displaypollingDaAggiornare=false;
}


//algoritmo 11
// chiamato ad ogni giro di poll
void MostraRisultatiVoto() {
  tft.fillRect(0,17,320,223,COLORESFONDO);
  tft.setCursor(0, 18);
  tft.setTextSize(3);
  for(int f=0;f<5;f++) {
    
    if(slavemigliorvoto[f]!=0) {
      if(f==0) tft.setTextColor(GREEN);  else tft.setTextColor(BLUE);  
      int ovi=slavemigliorvoto[f]->oravoto/1000000;
      int ovd=(slavemigliorvoto[f]->oravoto-ovi*1000000)/100;
      char str[25];
      sprintf(str, " %3d %3d,%04d",slavemigliorvoto[f]->indirizzo,ovi,ovd);
      tft.println(str);
    }
  }
}

void stampapkt(byte SENDERID,byte TARGETID, byte *DATA,int DATALEN) {
Serial.print(F("rxFrame: time/sender/target/dati: "));
	  Serial.print(micros());
	  Serial.print("/");
	  Serial.print(SENDERID);
	  Serial.print("/");
	  Serial.print(TARGETID);
	  Serial.print("/D:");
	  for (uint8_t i = 0; i < DATALEN; i++){
	      Serial.print(DATA[i],HEX);
		  Serial.print("/");
  	
	  }
	  Serial.println("");    
}