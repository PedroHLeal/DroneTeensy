#include <Arduino.h>
#include "controller_readings.h"
#include "bluetooth.h"

#define HWSERIAL Serial1

char btReading[20];
int btCurrent = 0;
int readValues[4];

void getRemoteCommands(ControllerReadings *r)
{
  while (HWSERIAL.available())
  {
    btReading[btCurrent] = HWSERIAL.read();
    // Serial.println(btReading[btCurrent]);
    btCurrent++;

    if (btReading[btCurrent - 1] == '\n')
    {
      int c = 0;
      char value[6];

      // reading throttle
      int v = 0;
      while (c < btCurrent)
      {
        if (btReading[c] != ' ')
        {
          value[v] = btReading[c];
        }
        else
        {
          value[v] = '\n';
          break;
        }
        v++;
        c++;
      }

      r->targetThrottle = String(value).toInt();

      // reading pitch
      v = 0;
      c++;
      while (c < btCurrent)
      {
        if (btReading[c] != ' ')
        {
          value[v] = btReading[c];
        }
        else
        {
          value[v] = '\n';
          break;
        }
        v++;
        c++;
      }
      r->setPointPitch = (-String(value).toInt()) * 2;

      // reading roll
      v = 0;
      c++;
      while (c < btCurrent)
      {
        if (btReading[c] != ' ')
        {
          value[v] = btReading[c];
        }
        else
        {
          value[v] = '\n';
          break;
        }
        v++;
        c++;
      }
      r->setPointRoll = (String(value).toInt()) * 2;
      // Serial.println(String(value).toInt());

      // reading ARM
      v = 0;
      c++;
      while (c < btCurrent)
      {
        if (btReading[c] != ' ')
        {
          value[v] = btReading[c];
        }
        else
        {
          value[v] = '\n';
          break;
        }
        v++;
        c++;
      }
      r->armed = (String(value).toInt()) == 1800;
      
      r->desiredHeight += r->targetThrottle * 0.5;

      // Serial.println(String(r->targetThrottle) + " " + String(r->setPointPitch) + " " + String(r->setPointRoll) + " " + String(r->armed));
      
      btCurrent = 0;
      break;
    }
  }
}