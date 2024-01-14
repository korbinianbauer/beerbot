/*
  Brian R Taylor
  brian.taylor@bolderflight.com

  Copyright (c) 2022 Bolder Flight Systems Inc

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the “Software”), to
  deal in the Software without restriction, including without limitation the
  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  sell copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.
*/

#include "sbus.h"

#define M_LEFT_DIR 4
#define M_LEFT_PWM 2
#define M_RIGHT_DIR 5
#define M_RIGHT_PWM 3

#define SBUS_CH_THROTTLE 1
#define SBUS_CH_STEER 0

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2);
/* SBUS data */
bfs::SbusData data;

int throttle = 0;
int steer = 0;

void setup() {
  pinMode(M_LEFT_DIR, OUTPUT);
  pinMode(M_LEFT_PWM, OUTPUT);
  pinMode(M_RIGHT_DIR, OUTPUT);
  pinMode(M_RIGHT_PWM, OUTPUT);

  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();


}

void loop () {
  if (sbus_rx.Read()) {

    data = sbus_rx.data();

    throttle = map(data.ch[SBUS_CH_THROTTLE], 174, 1811, -100, 100);
    steer = map(data.ch[SBUS_CH_STEER], 174, 1811, -100, 100);

    Serial.print(throttle);
    Serial.print("\t");
    Serial.println(steer);
  }
}
