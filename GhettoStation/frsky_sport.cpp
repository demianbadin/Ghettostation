#ifdef PROTOCOL_SPORT

void processFrskyPacket(uint8_t *packet);
void parseTelemHubByte(uint8_t c);
int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap);

#define START_STOP         0x7e
#define BYTESTUFF          0x7d
#define STUFF_MASK         0x20

// FrSky PRIM IDs (1 byte)
#define DATA_FRAME         0x10

// FrSky old DATA IDs (1 byte)
#define GPS_ALT_BP_ID      0x01
#define GPS_ALT_AP_ID      0x09
#define BARO_ALT_BP_ID     0x10
#define BARO_ALT_AP_ID     0x21
#define GPS_LONG_BP_ID     0x12
#define GPS_LONG_AP_ID     0x1A
#define GPS_LAT_BP_ID      0x13
#define GPS_LAT_AP_ID      0x1B
#define GPS_LONG_EW_ID     0x22
#define GPS_LAT_NS_ID      0x23
#define FRSKY_LAST_ID      0x3F
//used for sats and fix type
#define TEMP2              0x05

// FrSky new DATA IDs (2 bytes)
#define ALT_FIRST_ID       0x0100
#define ALT_LAST_ID        0x010f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f


#define TELEMETRY_INIT    0
#define TELEMETRY_OK      1
#define TELEMETRY_KO      2

#define FRSKY_RX_PACKET_SIZE 9 //19

uint8_t frskyRxBuffer[FRSKY_RX_PACKET_SIZE];
uint8_t telemetryState = TELEMETRY_INIT;

//alt in m
int16_t alt;
uint8_t sats;
uint8_t fix;

uint8_t HDOP;


uint16_t NS;
uint16_t EW;

//lat lon in decimal degree dd.ddddd
uint16_t lat_bp;
uint16_t lon_bp;
uint16_t lat_ap;
uint16_t lon_ap;

int16_t getFix() {
  return (uint8_t)fix;
}
int16_t getSats() {
  return (int16_t)sats;
}
int16_t getHDOP() {
  return (int16_t)HDOP;
}

int32_t gpsToLong(int8_t neg, uint16_t bp, uint16_t ap) {
  // we want convert from frsky to millionth of degrees
  // 0302.7846 -> 03 + (02.7846/60) = 3.04641 -> 3046410
  // first the upper part
  uint32_t first = ((uint32_t)bp / 100) * 100000;
  uint32_t second = ((((uint32_t)bp % 100) * 100000) + ((uint32_t)ap * 10)) / 60;

  // take sign into account
  return ((int32_t)(first + second) * (uint32_t)neg);
}

int32_t getTargetLat() {
  int32_t value = gpsToLong(NS == 'N' ? 1 : -1, lat_bp, lat_ap);
  NS = 0;
  return value;
}

int32_t getTargetLon() {
  int32_t value = gpsToLong(EW == 'E' ? 1 : -1, lon_bp, lon_ap);
  EW = 0;
  return value;
}

int16_t getTargetAlt() {
  return alt;
}

void processHubPacket(uint8_t id, uint16_t value)
{
  if (id > FRSKY_LAST_ID)
    return;
  switch (id) {
    case GPS_ALT_BP_ID:
      alt = value;
      break;
    case GPS_LONG_BP_ID:
      lon_bp = value;
      break;
    case GPS_LONG_AP_ID:
      lon_ap = value;
      break;
    case GPS_LAT_BP_ID:
      lat_bp = value;
      break;
    case GPS_LAT_AP_ID:
      lat_ap = value;
      break;
    case GPS_LONG_EW_ID:
      EW = value;
      break;
    case GPS_LAT_NS_ID:
      NS = value;
      break;
    case TEMP2:
      sats = value % 100;
      fix = bitRead(value / 1000, 0); //1 = GPS fix, 2 = GPS home fix, 4 = home reset (numbers are additive)
      HDOP = (value / 100) % 10; // GPS accuracy based on HDOP (0 = lowest to 9 = highest accuracy)
      break;
  }
  if ((NS == 'N' || NS == 'S') && (EW == 'E' || EW == 'W')) {
      //HAS_FIX = true;
  }
}

bool checkSportPacket(uint8_t *packet)
{
  short crc = 0;
  for (int i = 1; i < FRSKY_RX_PACKET_SIZE; i++) {
    crc += packet[i]; //0-1FF
    crc += crc >> 8; //0-100
    crc &= 0x00ff;
    crc += crc >> 8; //0-0FF
    crc &= 0x00ff;
  }
  return (crc == 0x00ff);
}

#define SPORT_DATA_U8(packet)   (packet[4])
#define SPORT_DATA_S32(packet)  (*((int32_t *)(packet+4)))
#define SPORT_DATA_U32(packet)  (*((uint32_t *)(packet+4)))
#define HUB_DATA_U16(packet)    (*((uint16_t *)(packet+4)))

void processSportPacket(uint8_t *packet)
{
  uint8_t  prim   = packet[1];
  uint16_t appId  = *((uint16_t *)(packet + 2));

  if (!checkSportPacket(packet))
    return;
  
  // agregado DMB
  //En caso de telegrama recibido ok ->
  telemetry_ok = true;
  lastpacketreceived = millis();
  protocol = "SPORT";
  //Serial.println("Telegrama recibido"); 
  
  switch (prim)
  {
    case DATA_FRAME:
      if ((appId >> 8) == 0) {
        // The old FrSky IDs
        uint8_t  id = (uint8_t)appId;
        uint16_t value = HUB_DATA_U16(packet);
        processHubPacket(id, value);
      }
      else if (appId >= T2_FIRST_ID && appId <= T2_LAST_ID) {
          sats = SPORT_DATA_S32(packet) % 100;
          fix = bitRead(SPORT_DATA_S32(packet) / 1000, 0); //1 = GPS fix, 2 = GPS home fix, 4 = home reset (numbers are additive)
          HDOP = (SPORT_DATA_S32(packet) / 100) % 10; // GPS accuracy based on HDOP (0 = lowest to 9 = highest accuracy)
      }
      else if (appId >= GPS_SPEED_FIRST_ID && appId <= GPS_SPEED_LAST_ID) {
        //frskyData.hub.gpsSpeed_bp = (uint16_t) (SPORT_DATA_U32(packet) / 1000);
      }
      else if (appId >= GPS_COURS_FIRST_ID && appId <= GPS_COURS_LAST_ID) {
        //uint32_t course = SPORT_DATA_U32(packet)/100;
      }
      #ifdef BARO_ALT
        else if (appId >= BARO_ALT_FIRST_ID && appId <= BARO_ALT_LAST_ID) {
          alt = SPORT_DATA_S32(packet); // altura en cm
        }
      #else
        else if (appId >= GPS_ALT_FIRST_ID && appId <= GPS_ALT_LAST_ID) {
          alt = SPORT_DATA_S32(packet); // altura en cm
        }
      #endif
      else if (appId >= GPS_LONG_LATI_FIRST_ID && appId <= GPS_LONG_LATI_LAST_ID) {
        uint32_t gps_long_lati_data = SPORT_DATA_U32(packet);
        uint32_t gps_long_lati_b1w, gps_long_lati_a1w;
        gps_long_lati_b1w = (gps_long_lati_data & 0x3fffffff) / 10000;
        gps_long_lati_a1w = (gps_long_lati_data & 0x3fffffff) % 10000;

        switch ((gps_long_lati_data & 0xc0000000) >> 30) {
          case 0:
            lat_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
            lat_ap = gps_long_lati_a1w;
            NS = 'N';
            break;
          case 1:
            lat_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
            lat_ap = gps_long_lati_a1w;
            NS = 'S';
            break;
          case 2:
            lon_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
            lon_ap = gps_long_lati_a1w;
            EW = 'E';
            break;
          case 3:
            lon_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
            lon_ap = gps_long_lati_a1w;
            EW = 'W';
            break;
        }
        if ((NS == 'N' || NS == 'S') && (EW == 'E' || EW == 'W')) {
          //HAS_FIX = true;
        }
      }
      break;
  }
}

// Receive buffer state machine state enum
enum FrSkyDataState {
  STATE_DATA_IDLE,
  STATE_DATA_IN_FRAME,
  STATE_DATA_XOR,
};

void encodeTargetData(uint8_t data) {
  static uint8_t numPktBytes = 0;
  static uint8_t dataState = STATE_DATA_IDLE;

  if (data == START_STOP) {
    dataState = STATE_DATA_IN_FRAME;
    numPktBytes = 0;
  }
  else {
    switch (dataState) {
      case STATE_DATA_XOR:
        frskyRxBuffer[numPktBytes++] = data ^ STUFF_MASK;
        dataState = STATE_DATA_IN_FRAME;
        break;

      case STATE_DATA_IN_FRAME:
        if (data == BYTESTUFF)
          dataState = STATE_DATA_XOR; // XOR next byte
        else
          frskyRxBuffer[numPktBytes++] = data;
        break;
    }
  }

  if (numPktBytes == FRSKY_RX_PACKET_SIZE) {
    processSportPacket(frskyRxBuffer);
    dataState = STATE_DATA_IDLE;
  }
}
int sport_read(void) {
/*
  //aca viene la función de lectura de datos Sport
  //Datos necesarios para Ghetto:
  //uav_groundspeed = (uint16_t) round((float)(uav_groundspeedms * 3.6f)); // convert to kmh
  //uav_alt = (int32_t)ltmread_u32();
  //uav_satellites_visible         = (ltm_satsfix >> 2) & 0xFF;
  //uav_fix_type                   = ltm_satsfix & 0b00000011;
  //uav_lat = (int32_t)ltmread_u32();
  //uav_lon = (int32_t)ltmread_u32();

  //En caso de telegrama recibido ok ->
  //telemetry_ok = true;
  //lastpacketreceived = millis();
  //protocol = "SPORT";

*/
  int incomingByte = 0;   // for incoming serial data

  if (Serial1.available() > 0) {
                // read the incoming byte:
                uint8_t c = char( Serial1.read());
                encodeTargetData(c);                
}

  uav_lat = getTargetLat()*100;
  uav_lon = getTargetLon()*100;
  uav_alt = getTargetAlt();
  uav_satellites_visible = getSats();
  uav_fix_type = getFix();
}


    
#endif
