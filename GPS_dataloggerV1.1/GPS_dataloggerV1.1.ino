/*
    GPS Data correction, SD write, Temperature, PinEnabled, Magnet detect
    Alpha Version
*/
#include <SD.h>
#include <SPI.h>
#include <OneWire.h>
#include "DallasTemperature.h"
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include "LowPower.h"


const String DEVICE_TAG = "TAG123";     // Device Tag (AlphaNum) AT most 6 letter/number

#define STOP_PIN 2  // Stop Device (Reed Switch)
#define GPS_EN 5    // GPS Enable pin
#define SD_EN 6     // SD card Enable Pin
#define RESET_DEV A3    // Pin A3 to reset the device
#define ONE_WIRE_BUS 7  // Temp Pin
#define SD_INIT_CHK 3   // SD card initialization limit (how many times)
#define GPS_CHK_MIN 4   // Wait for signal from GPS in minute
#define DEEP_SLEEP_SEC  14400     // EX: 2 hours = 60x60x2 = 7200 s

double getTemperature(void);     // Read temp
String getGPS(void);            // Read GPS 
int storeData(String *data);    // Store Data
void stop_device(void);         // ISR for Mag Switch

SoftwareSerial serial_connection(4, 3); //RX=pin 4, TX=pin 3
TinyGPSPlus gps;    //This is the GPS object that will pretty much do all the grunt work with the NMEA data

OneWire oneWire(ONE_WIRE_BUS);  // For Temperature
DallasTemperature temp_sensor(&oneWire);

const int chip_select = 10;     // SD card Chip select
volatile bool reset_now = false;    // True if magnet is attached: involved main loop & getGPS

void setup()
{
    // Configure STOP Pin and Interrupt
    pinMode(STOP_PIN, INPUT_PULLUP);    // Normally HIGH
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), stop_device, FALLING);
    
    // LED check
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);     // Indicator LED on

    // Start Configuration
    digitalWrite(GPS_EN, HIGH);   
    digitalWrite(SD_EN, HIGH);
    pinMode(GPS_EN, OUTPUT);
    pinMode(SD_EN, OUTPUT);

    delay(2000);
    digitalWrite(GPS_EN, HIGH);   
    digitalWrite(SD_EN, HIGH);

    Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE

    digitalWrite(13, LOW);     // Indicatior LED off
    pinMode(13, INPUT);

    delay(500);
    Serial.println("Start device");
    delay(500);
}
// float temperature=0;
void loop()
{
    digitalWrite(GPS_EN, HIGH);   
    digitalWrite(SD_EN, HIGH);

    // If reset_now is not triggered and STOP_PIN is high (Magnet not attached)
    if (!reset_now && digitalRead(STOP_PIN)){       
        String gps_data = "";

        // Get Temperature and GPS Data
        Serial.println("Get Temp & GPS Data");
        gps_data = getGPS();
        Serial.println(gps_data);

        // Check if GPS data is available
        if (gps_data == "")                 // If data is not available
            Serial.println("No data");
        else{
            Serial.println("Store Data");
            if(!storeData(&gps_data))
                Serial.println("Error Writing data");
        }
        // Serial.println("Data Available");

    
        // /*      Low Power Sleep 
        Serial.println("Entering Low Power");
        delay(100);

        for (unsigned int sleepCounter = DEEP_SLEEP_SEC; sleepCounter > 0; sleepCounter--)
        {
            LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);  
            if (reset_now)
                break;
        }
        // Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
        // delay(800);
        Serial.println("Low Power Ends");
        // Low Power Sleep Ends    */  
    }

    else{
        Serial.println("Reset: Magnet Attached");
        delay(100);
        // pinMode(STOP_PIN, INPUT);
        // digitalWrite(STOP_PIN, HIGH);
        while (!digitalRead(STOP_PIN))
        {
            pinMode(STOP_PIN, INPUT);           // PULLUP consumes much power, change to INPUT
            LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);  
            pinMode(STOP_PIN, INPUT_PULLUP);    // Check for the Magnet, changing to PULLUP
        }
        // pinMode(STOP_PIN, INPUT_PULLUP);
        reset_now = false;
    } 

    // delay(2000);

}//End Loop


// Get Temperature (in F)
double getTemperature(void){
    int count = 0;
    double temp;
    temp_sensor.begin();        // Sensor reading begins
    delay(100);
    while(count < 20){
        temp_sensor.requestTemperatures();
        temp = temp_sensor.getTempFByIndex(0);
        if(temp > -50 && temp < 130)
            return temp;
        count++;
    }
    return 185;
}


// Get GPS Data
String getGPS(void){
    unsigned long int start_time=millis(), interval = GPS_CHK_MIN * 60000;
    double lat_stored[5] = {0.0}, lon_stored[5] = {0.0};     // Store GPS data for accuracy
    double lat_calc=0, lon_calc=0;                          // Calculated lat and lon
    double temperature = 0;
    uint8_t count = 0;                            // Count how many times data received
    uint8_t store_from=20, store_to=25;                     // Analyze data from and to
    // Serial.println("Get GPS data");
    // Configuration: Start GPS
    digitalWrite(GPS_EN, LOW);   
    serial_connection.begin(9600);//This opens up communications to the GPS

    delay(500);
    // Collect Temperature in this window (GPS wait time)
    temperature = getTemperature();

    // Check for the GPS signal for GPS_CHK_MILLISECOND
    while(millis() - start_time <= interval && !reset_now){      
        while(serial_connection.available())//While there are characters to come from the GPS
            gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time

        // Store GPS Data and check if it is not first data
        if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
        {
            interval = GPS_CHK_MIN * 60000 + 30000;     // Add 30sec to timeout interval

            count++;
            Serial.print("Count: ");
            Serial.println(count);
            
            // Store location for count number "store_from" to count number "store_to"
            if (count < store_from){
                Serial.println(gps.location.lat(), 6);
                Serial.println(gps.location.lng(), 6);
            }

            else if (count >= store_from && count < store_to){
                lat_stored[count - store_from] = gps.location.lat();     // "lat_store" Index from zeor
                lon_stored[count - store_from] = gps.location.lng();
                Serial.println(lat_stored[count - store_from], 6);
                Serial.println(lon_stored[count - store_from], 6);
            }

            // If count >= 15, Turn off GPS & Calculate final lat and lon
            else {
                // Trun off GPS module and return data
                digitalWrite(GPS_EN, HIGH);   

                // Calculate Lat & Lon
                for (int i=0; i<(store_to - store_from); i++){      // i from 0 to (store_to-store_from)
                    lat_calc += lat_stored[i];
                    lon_calc += lon_stored[i];
                }
                lat_calc /= (store_to - store_from);    // Find mean of stred lats
                lon_calc /= (store_to - store_from);

                // Convert GPS data into string 
                // Format: year,month,day,hour,minute,second,latitude,longitude
                String gps_data = "";
                gps_data += String(gps.date.year()) + ",";
                gps_data += String(gps.date.month()) + ",";
                gps_data += String(gps.date.day()) + ",";
                gps_data += String(gps.time.hour()) + ",";
                gps_data += String(gps.time.minute()) + ",";
                gps_data += String(gps.time.second()) + ",";
                
                gps_data += String(lat_calc, 6) + ",";
                gps_data += String(lon_calc, 6) + ",";
                // Serial.println(gps_data);

                // If temperature was not read properly
                if (temperature == 185.00)
                    temperature = getTemperature();

                // Include Temperature into gps data
                gps_data += String(temperature, 1);

                return gps_data;    // Return GPS & Temperature
            }
            delay(800);
        }//End If
    }//Loop GPS data acquisition
    
    // If GPS is not read within the time limit, return empty string and turn off GPS Module
    digitalWrite(GPS_EN, HIGH);   
    Serial.println("GPS Timeout!!!");
    return String("");
}


// Store data in SD card
int storeData(String *data){
    // Initialize SD Card
    digitalWrite(SD_EN, LOW);       // Turn on SD card
    Serial.println("Initializing SD card...");
    // If SD card initialization fails, check for SD_INIT_CHK times
    for (int count_sd=0; count_sd<SD_INIT_CHK && !SD.begin(chip_select); count_sd++) {
        digitalWrite(SD_EN, HIGH);
        delay(1000);
        digitalWrite(SD_EN, LOW);
        if (count_sd = SD_INIT_CHK-1){
            Serial.println("SD card initialization failed");
            return 0;
        }
    }//for SD card check
    Serial.println("SD Card Ready");
    delay(100);

    // Open the output file & Write
    String file_name = "G_" + DEVICE_TAG + ".csv";
    File dataFile = SD.open(file_name, FILE_WRITE);
    // If file is available
    if(dataFile){
        dataFile.println(*data);
        dataFile.close();
        Serial.print("Written: ");
        Serial.println(*data);          //Write to serial
    }
    // If file not available
    else 
        Serial.println("Error opening file");

    // Unmount SD Card
    digitalWrite(SD_EN, HIGH);

    return 1;
}


// Reset device
void stop_device(void){
    // Update Reset variable
    reset_now = true;                   // Affect Mainloop & getGPS()
    delay(500);         // Debouncing delay
}
