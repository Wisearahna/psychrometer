#include <OneWire.h>
#include <QuadDisplay2.h>
#include <math.h>

//constants ГОСТ8.524-85
#define G 6888.2
#define K 273.15
#define D -5.3627
#define e 2.718
#define PK 10

//QuadDisplay chip select on pin 9
QuadDisplay qd(9);
//two dallas DS18B20 on pin 10 (4.7K resistor pull up)
OneWire  ds(10);

//vars
byte i;
//byte present = 0;
byte type_s = 0;  //DS18B20
byte data1[12];
byte data2[12];
byte addr1[8];
byte addr2[8];
float t, td, rh;
int16_t ti;

void setup(void)
{
  //Serial.begin(9600);
  qd.begin();

// search both dallas chips
  if ( !ds.search(addr1)) {
    ds.reset_search();
    delay(250);
    return;
  }
  if ( !ds.search(addr2)) {
    ds.reset_search();
    delay(250);
    return;
  }
/*
//check CRC
  if (OneWire::crc8(addr, 7) != addr[7]) {
    //Serial.println("CRC is not valid!");
    return;
  }

//check chip type
// the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  }
*/

}

void loop(void)
{
  //read dallas chips
  td = (float)get_temperature(addr1, data1) / 16.0;
  ti = get_temperature(addr2, data2) / 16.0;
  t=(float)ti;
  //ГОСТ8.524-85 formula (3)
  rh = 100 * pow(K + td, D) * pow(K + t, -D) * pow(e, G * ((1 / (K + t)) - (1 / (K + td))))-PK;
/*  
  //print
  Serial.print(" td, t, d: ");
  Serial.print(t);
  Serial.print(" ; ");
  Serial.print(td);
  Serial.print(" ; ");
  Serial.print(t-td);
  Serial.print(" Rel. Hum. % = ");
  Serial.println(rh);
*/
  //display
  qd.displayTemperatureC(ti, false);
  delay(3000);
  qd.displayHumidity((int)rh, false);
  delay(1000);
}

//
int16_t get_temperature(byte *addr, byte *data)
{
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);  // start conversion, with parasite power on at the end
  
  delay(1000);  // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it
    
  ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad
  for ( i = 0; i < 9; i++) {  // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) { //DS18S20
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {  //DS18B20
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  return raw;
}
