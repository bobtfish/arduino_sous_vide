
// Pin 4 has an LED + triac driver connected.
int AC_LOAD = 4;

/* Pin    |  Interrrupt # | Arduino Platform
   ---------------------------------------
   2      |  0            |  All
   3      |  1            |  All
*/
// Pin 3 has our zero crossing detector
int ZERO_CROSS = 1; // But note this is the interrupt #

int dimming = 128;  // Dimming level (0-128)  0 = ON, 128 = OFF



// the setup routine runs once when you press reset:
void setup() {
  pinMode(AC_LOAD, OUTPUT);
  attachInterrupt(ZERO_CROSS, zero_crosss_int, RISING);
}

// the interrupt function must take no parameters and return nothing
void zero_crosss_int() {
// function to be fired at the zero crossing to dim the light
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
  for (int i=5; i <= 128; i++) {
    dimming=i;
    delay(10);
  }
}

