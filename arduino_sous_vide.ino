#include <OneWire.h>

// Pin 4 has an LED + triac driver connected.
int AC_LOAD = 4;

/* Pin    |  Interrrupt # | Arduino Platform
   ---------------------------------------
   2      |  0            |  All
   3      |  1            |  All
*/
// Pin 3 has our zero crossing detector
int ZERO_CROSS = 1; // But note this is the interrupt #

int dimming = 5;  // Dimming level (0-128)  5 = ON, 128 = OFF

float desired_temp = 25.0;

int has_seen_interrupt = 0;

OneWire ds(10);

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  pinMode(AC_LOAD, OUTPUT);
  digitalWrite(AC_LOAD, HIGH);
  attachInterrupt(ZERO_CROSS, zero_crosss_int, RISING);
}

// the interrupt function must take no parameters and return nothing
void zero_crosss_int() {
  // function to be fired at the zero crossing to dim the light
  has_seen_interrupt = 1;
  // Firing angle calculation : 1 full 50Hz wave =1/50=20ms
  // Every zerocrossing thus: (50Hz)-> 10ms (1/2 Cycle) For 60Hz => 8.33ms

  // 10ms=10000us
  // (10000us - 10us) / 128 = 75 (Approx) For 60Hz =>65
  int dimtime = (75*dimming);    // For 60Hz =>65
  delayMicroseconds(dimtime);    // Off cycle
  digitalWrite(AC_LOAD, HIGH);   // triac firing
  delayMicroseconds(10);         // triac On propogation delay
                                 //(for 60Hz use 8.33)
  digitalWrite(AC_LOAD, LOW);    // triac no longer firing
}

// For testing, we just repeatedly cycle the dimming over from dim to bright
void loop()  {
  float current_temp;
    current_temp = get_temp();
    Serial.print("Current temp: ");
    Serial.println(current_temp);
    Serial.print("Desired temp: ");
    Serial.println(desired_temp);
    Serial.print("Adjustment direction ");
    if (current_temp > desired_temp) {
      Serial.println(">");
      dimming++;
    }
    else {
      Serial.println("<");
      dimming--;
    }
    if (dimming < 5 )
      dimming = 5;
    if (dimming > 128)
      dimming = 128;
      
    Serial.print("Dimming level (128 = OFF, 5 = ON): ");
    Serial.println(dimming);
    Serial.println("---------------------------------");
    
}


float get_temp(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  
  if ( !ds.search(addr) ) {
    ds.reset_search();
    delay(250);
    ds.search(addr);
  }
  
  /*Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }*/

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -3;
  }
  //Serial.println();
 
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
      return -1;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  return celsius;
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.println(" Celsius");
}
