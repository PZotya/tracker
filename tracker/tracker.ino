#include <FSKModem.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <EEPROM.h>

HMC5883L compass;
LiquidCrystal lcd(4, 5, 6, 7, 8, 9);

Servo horizontal;
Servo vertical;

struct settings {
  float offset;
  bool vertInv;
  bool horInv;
};

settings saved;
byte i, j, statusIndex;
byte rec, bitnum, menuitem;
bool locked, menubutton, haspacket, selected;
int lastSatnum,satnum;
float fix_alt, fix_long, fix_lat, curr_alt, curr_long, curr_lat, course_angle, cheading, heading, vertical_angle;
unsigned long fix_age, curr_age, pressTime;
TinyGPS gps;
char progress[5] = "*|/-" ;
Vector norm;
//char message[18]="0123456789ABCDEF";


void setup() {
  lcd.begin(16, 2);
  lcd.print("Tracker Init");
  lcd.setCursor(0, 1);
  EEPROM.get(0, saved);
  lcd.print("."); 
  modem.begin(1200, 0, 2);
  lcd.print("."); 


  /*Serial.begin(9600);
    Serial.println("START");*/
  pinMode(10, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  lcd.print("."); 
  horizontal.attach(11, 1000, 2000);
  vertical.attach(12, 1000, 2000);
  horizontal.write(90);
  vertical.write(90);
  lcd.print("."); 
  compass.begin();
  //Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  //Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
  lcd.print("."); 
  rec = 0;
  bitnum = 0;
  locked = false;
  menuitem = 0;
  menubutton = false;
  haspacket = false;
  statusIndex = 0;
  selected= false;
  lcd.clear();
  lcd.print("GPS Info  SAT:");
  if (digitalRead(10) == 1) {
    digitalWrite(13, HIGH);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (menubutton) {
    if (digitalRead(10) == 1) {
      digitalWrite(13, HIGH);
      menubutton = false;
      if (millis() < pressTime + 1500)
      { menuitem++;
        if (menuitem > 6) menuitem = 0;
        lcd.clear();
        switch (menuitem) {
          case 0:
            lcd.print("GPS Info  SAT:");
            selected=true;
            break;
          case 1:
            lcd.print("Compass  value  ");
            break;
          case 2:
            lcd.print("FSK Buffer   :  ");
            break;
          case 3:
            lcd.print("Distance:");
            break;
          case 4:
            lcd.print("Direction:");
            break;
          case 5:
            lcd.print("Horiz. Servo:");
            break;
          case 6:
            lcd.print("Vert. Servo:");
            break;

        }
      } else
      {
        switch (menuitem) {
          case 0:
            saved.vertInv = false;
            saved.horInv = false;
            saved.offset = 0;
            EEPROM.put(0, saved);
            break;
          case 1:
            saved.offset = cheading;
            EEPROM.put(0, saved);
            break;
          case 5:
            saved.horInv = !saved.horInv;
            EEPROM.put(0, saved);
            break;
          case 6:
            saved.vertInv = !saved.vertInv;
            EEPROM.put(0, saved);
            break;
        }

      }
    }
  } else {
    if (digitalRead(10) == 0) {
      menubutton = true;
      pressTime = millis();
      digitalWrite(13, LOW);
    }
  }


  norm = compass.readNormalize();
  cheading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (4.0 + (34.0 / 60.0)) / (180 / M_PI);
  cheading += declinationAngle;
  if (cheading < 0) {
    cheading += 2 * PI;
  }
  if (cheading > 2 * PI) {
    cheading -= 2 * PI;
  }
  
  heading = cheading - saved.offset;
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  heading *= 180 / PI;




  switch (menuitem) {
    case 0:
      lcd.setCursor(14, 0);
      if (haspacket) {
        if (lastSatnum!=satnum)
        {
          lcd.print(satnum);
          lastSatnum=satnum;
        }
      } else {
        if (((millis() / 200) % 4)!=statusIndex){
          statusIndex=(millis() / 200) % 4;
          lcd.print(progress[statusIndex]);
        }
      }
      if (locked & selected) {
        lcd.setCursor(0, 1);
        lcd.print("LN"); lcd.print(curr_long);
        lcd.print(" LT"); lcd.print(curr_lat);
        selected=false;
      }
      break;
    case 1:
      if (millis() % 100 == 0) {

        lcd.setCursor(0, 1);
        lcd.print("     ");
        lcd.setCursor(0, 1);
        lcd.print(round(heading));
      }
      break;
    case 2:
      lcd.setCursor(0, 1);
      for (i = 0; i < 16; i++) {
        lcd.write(modem.status(i));
      }
      break;
    case 3:
      lcd.setCursor(0, 1);
      if (locked) {
        lcd.print(gps.distance_between (fix_lat, fix_long, curr_lat, curr_long));
      }
      break;
    case 4:
      lcd.setCursor(0, 1);
      if (locked) {
        lcd.print(course_angle);
      }
      break;
    case 5:
      lcd.setCursor(0, 1);
      if (saved.horInv) {
        lcd.print("Inverse");
      } else {
        lcd.print("Normal ");
      }
      break;
    case 6:
      lcd.setCursor(0, 1);
      if (saved.vertInv) {
        lcd.print("Inverse");
      } else {
        lcd.print("Normal ");
      }
      break;
  }




  while (modem.available() > 0)
  {
    j = modem.read();
    // Serial.write(j);
    if (gps.encode(j)) {
      haspacket = true;
      satnum = gps.satellites();
      if (locked)
      {
        gps.f_get_position(&curr_long, &curr_lat, &curr_age);
        if (gps.distance_between (fix_lat, fix_long, curr_lat, curr_long) > 6)
        {
          curr_alt = gps.altitude() / 100;
          course_angle = gps.course_to (fix_lat, fix_long, curr_lat, curr_long) ;
          course_angle -= heading;
          if (course_angle < 0) {
            course_angle += 360;
          }
          vertical_angle = atan2(curr_alt - fix_alt, gps.distance_between (fix_lat, fix_long, curr_lat, curr_long)) * 180.0 / PI;
          if (course_angle > 180) {
            if (saved.horInv) {
              horizontal.write(360 - course_angle);
            } else {
              horizontal.write(course_angle - 180);
            }
            if (saved.vertInv) {
              vertical.write(vertical_angle);
            } else {
              vertical.write(180 - vertical_angle);
            }

          } else {

            if (saved.horInv) {
              horizontal.write(180 - course_angle);
            } else {
              horizontal.write(course_angle);
            }
            if (saved.vertInv) {
              vertical.write(180 - vertical_angle);
            } else {
              vertical.write(vertical_angle);
            }


            //            horizontal.write(course_angle);
            //            vertical.write(vertical_angle);
          }
          /*      Serial.print(" Adatok ");
                Serial.print(course_angle);
                Serial.print(" fok irány ");
                Serial.print(gps.distance_between (fix_lat,fix_long,curr_lat, curr_long));
                Serial.print(" távolság ");
                Serial.print(curr_alt-fix_alt);
                Serial.print(" magasság ");
                Serial.print(atan2(curr_alt-fix_alt,gps.distance_between (fix_lat,fix_long,curr_lat, curr_long))*180.0/PI);
                Serial.print(" vert szög, ");
                Serial.print("mért magasság: ");
                  Serial.println(curr_alt);*/
        } else {
          /*Serial.println(" Közel!"); */
          horizontal.write(90);
          vertical.write(90);
        }
      } else {
        /*        Serial.print(satnum);
                Serial.println(" GPS SAT");*/
        if (satnum > 5) {
          locked = true;
          fix_alt = gps.altitude() / 100;
          /*          Serial.print("Rögzített magasság: ");
                   Serial.println(fix_alt); */
          gps.f_get_position(&fix_long, &fix_lat, &fix_age);
        }
      }
    }
  }


}
