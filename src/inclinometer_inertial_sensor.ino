#include <WiFi.h>
#include <ESP32_FTPClient.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <MPU9250_WE.h>
#include "Arduino.h"
#include <ModbusIP_ESP8266.h>

File myFile;

const int REG = 0;               // Modbus Hreg Offset
IPAddress remote(192, 168, 4, 1);  // Address of Modbus Slave device
const int LOOP_COUNT = 10;
uint16_t res = 0;
uint8_t show = LOOP_COUNT;
uint16_t old_res = 0;
bool conn;
int var = 0;
int inviato;

ModbusIP mb;  //ModbusIP object


#define WIFI_SSID "CICCIO4"
#define WIFI_PASS "nonserve"
IPAddress ip(192, 168, 4, 19);

#ifdef ESP32_HAL_I2C_H
#define SDA_PIN 21
#define SCL_PIN 22
#endif

char ftp_server[] = "192.168.4.1";
char ftp_user[] = "pi";
char ftp_pass[] = "nonserve";

// you can pass a FTP timeout and debbug mode on the last 2 arguments
ESP32_FTPClient ftp(ftp_server, ftp_user, ftp_pass, 7500, 2);

MPU9250_WE mySensor = MPU9250_WE(0X68);


float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ, off_ax, off_ay, off_az;



void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root) {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels) {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS& fs, const char* path) {
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path)) {
        Serial.println("Dir created");
    }
    else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS& fs, const char* path) {
    Serial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path)) {
        Serial.println("Dir removed");
    }
    else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS& fs, const char* path) {
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available()) {
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS& fs, const char* path, const char* message) {
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        Serial.println("File written");
    }
    else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS& fs, const char* path, const char* message) {
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message)) {
        Serial.println("Message appended");
    }
    else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS& fs, const char* path1, const char* path2) {
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    }
    else {
        Serial.println("Rename failed");
    }
}

void testFileIO(fs::FS& fs, const char* path) {
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if (file) {
        len = file.size();
        size_t flen = len;
        start = millis();
        while (len) {
            size_t toRead = len;
            if (toRead > 512) {
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    }
    else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for (i = 0; i < 2048; i++) {
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}


void setup() {

    Serial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASS);



    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    mb.client();

    Serial.println("started");

//#ifdef ESP32_HAL_I2C_H // For ESP32
//    Wire.begin(SDA_PIN, SCL_PIN);
//    mySensor.setWire(&Wire);
//#endif

    Wire.begin();
    if (!mySensor.init()) {
        Serial.println("MPU9250 does not respond");
    }
    else {
        Serial.println("MPU9250 is connected");
    }

    delay(1000);

    mySensor.autoOffsets();

    /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
 *  Sample rate = Internal sample rate / (1 + divider)
 *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
 *  Divider is a number 0...255
 */
    mySensor.setSampleRateDivider(5);

    /*  MPU9250_ACC_RANGE_2G      2 g    (default)
 *  MPU9250_ACC_RANGE_4G      4 g
 *  MPU9250_ACC_RANGE_8G      8 g
 *  MPU9250_ACC_RANGE_16G    16 g
 */
    mySensor.setAccRange(MPU9250_ACC_RANGE_2G);

    /*  Enable/disable the digital low pass filter for the accelerometer
 *  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
 */
    mySensor.enableAccDLPF(true);

    /*  Digital low pass filter (DLPF) for the accelerometer, if enabled
 *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7
 *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
 *     0           460               1.94           1
 *     1           184               5.80           1
 *     2            92               7.80           1
 *     3            41              11.80           1
 *     4            20              19.80           1
 *     5            10              35.70           1
 *     6             5              66.96           1
 *     7           460               1.94           1
 */
    mySensor.setAccDLPF(MPU9250_DLPF_6);



    Serial.println("Connecting Wifi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Connected");

    if (!SD.begin()) {
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }

}

void loop() {
    mb.task();                      // Common local Modbus task
    delay(100);
    
    
    if (mb.isConnected(remote)) {   // Check if connection to Modbus Slave is established

        mb.writeHreg(remote, REG+4, 99, NULL, 1);
        delay(100);
    }
    else {
        mb.connect(remote);           // Try to connect if no connection
        Serial.println("Tentativo di connessione");
    }


    mb.readIreg(remote, REG, &res);  // Initiate Read Coil from Modbus Slave
    delay(100);
    xyzFloat angles = mySensor.getAngles();
    float Pitch = mySensor.getPitch();
    float Roll = mySensor.getRoll();
    

    if (res == 2 && old_res == 0) {
        Serial.println("Calibrazione Avvenuta");
        //int num = REG + 1;
        mySensor.autoOffsets();
        //mb.writeHreg(remote, num, 3);
        mb.writeHreg(remote, REG + 1, 3, NULL, 1);
        delay(5000);
        ////mb.writeHreg(remote, num, 0);
        mb.writeHreg(remote, REG + 1, 0, NULL, 1);
    }




    Serial.print("Pitch:");
    Serial.println(Pitch);
    Serial.print("Roll:");
    Serial.println(Roll);
   
    mb.writeHreg(remote, REG + 2, (int)((Pitch+100) * 100));
    mb.writeHreg(remote, REG + 3, (int)((Roll+100) * 100));
    //mb.writeHreg(remote, REG + 4,(int)analogRead(0));

    Serial.print("Pitch:");
    Serial.println(Pitch);
    Serial.print("Roll:");
    Serial.println(Roll);
    //Serial.print("Batt:");
    //Serial.println(analogRead(0));


    delay(50);
    mb.writeHreg(remote, REG, 0, NULL, 1);
    delay(50);

    if (res == 1 && old_res == 0) {

        mb.writeHreg(remote, REG, 1,NULL,1);

        delay(1000);
        Serial.println("Start");
        myFile = SD.open("/dati.csv", FILE_WRITE);
        myFile.print("Millis");
        myFile.print(";");
        myFile.print("Ax");
        myFile.print(";");
        myFile.print("Ay");
        myFile.print(";");
        myFile.print("Az");
        myFile.print(";");
        myFile.print("aSqrt");
        myFile.print(";");
        myFile.print("Gx");
        myFile.print(";");
        myFile.print("Gy");
        myFile.print(";");
        myFile.print("Gz");
        myFile.print(";");
        myFile.print("Mx");
        myFile.print(";");
        myFile.print("My");
        myFile.print(";");
        myFile.print("Mz");
        myFile.print(";");
        myFile.print("mDirection");
        myFile.print("\n");
        //while (WiFi.status() != WL_CONNECTED){ 
        mb.writeHreg(remote, REG, 1, NULL, 1);
        for (int x = 0; x <= 2047; x++) {
            Serial.println(x);
            uint8_t sensorId;
            xyzFloat gValue = mySensor.getGValues();
            aX = gValue.x;
            aY = gValue.y;
            aZ = gValue.z;
            aSqrt = 0.0;
            gX =0.0;
            gY = mySensor.getPitch();
            gZ = mySensor.getRoll();
            mX = 0.0;
            mY = 0.0;
            mZ = 0.0;
            mDirection = mySensor.getTemperature();
            aX = aX * 1000;
            aY = aY * 1000;
            aZ = aZ * 1000;
            aSqrt = aSqrt * 1000;
            gX = gX * 1000;
            gY = gY * 1000;
            gZ = gZ * 1000;
            mX = mX * 1000;
            mY = mY * 1000;
            mZ = mZ * 1000;
            mDirection = mDirection * 1000;
            char saX[10];
            char saY[10];
            char saZ[10];
            char saSqrt[10];
            char sgX[10];
            char sgY[10];
            char sgZ[10];
            char smX[10];
            char smY[10];
            char smZ[10];
            char smDirection[10];
            itoa(aX, saX, 10);
            itoa(aY, saY, 10);
            itoa(aZ, saZ, 10);
            itoa(aSqrt, saSqrt, 10);
            itoa(gX, sgX, 10);
            itoa(gY, sgY, 10);
            itoa(gZ, sgZ, 10);
            itoa(mX, smX, 10);
            itoa(mY, smY, 10);
            itoa(mZ, smZ, 10);
            itoa(mDirection, smDirection, 10);
            myFile.print(millis());
            myFile.print(";");
            myFile.print(saX);
            myFile.print(";");
            myFile.print(saY);
            myFile.print(";");
            myFile.print(saZ);
            myFile.print(";");
            myFile.print(saSqrt);
            myFile.print(";");
            myFile.print(sgX);
            myFile.print(";");
            myFile.print(sgY);
            myFile.print(";");
            myFile.print(sgZ);
            myFile.print(";");
            myFile.print(smX);
            myFile.print(";");
            myFile.print(smY);
            myFile.print(";");
            myFile.print(smZ);
            myFile.print(";");
            myFile.print(smDirection);
            myFile.print("\n");
            delay(16);
        }
        myFile.close();
        Serial.println("libera");
        delay(7000);


        var = 0;
        do {
            if (WiFi.status() != WL_CONNECTED) {
                WiFi.begin(WIFI_SSID, WIFI_PASS);
            }
            while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                Serial.print(".");
            }

            Serial.println("Invio in ftp");


            do {
                ftp.OpenConnection();
                delay(1000);
                ftp.InitFile("Type A");
                delay(1000);
                ftp.ChangeWorkDir("files");
                delay(1000);
            } while (!ftp.isConnected() && (WiFi.status() == WL_CONNECTED));

            myFile = SD.open("/dati.csv");
            inviato = 0;


            ftp.NewFile("dati.csv");
            delay(2000);
            if (myFile) {
                Serial.println("dati.csv:");
                // read from the file until there's nothing else in it:
                while (myFile.available()) {
                    const char* dat;
                    String list = myFile.readStringUntil('\n');
                    //int str_len = list.length() + 1; 
                    Serial.println(list);
                    dat = list.c_str();
                    //list.toCharArray(dat, list.length());
                    Serial.println(dat);
                    ftp.Write(dat);
                    ftp.Write("\n");
                    delay(10);
                }


                // close the file:
                myFile.close();

                if (ftp.isConnected()) inviato = 1;
                
            }

            delay(1000);
            ftp.CloseFile();
            delay(1000);
            ftp.CloseConnection();
            delay(1000);
        } while (inviato == 0);
        
        
    }
    delay(50);
    old_res = res;
}
