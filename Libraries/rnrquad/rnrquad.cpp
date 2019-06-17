#include "rnrquad.h"
#include "state.h"

extern void baseSetup()
{
    pinMode(versionGPIO, INPUT_PULLUP);
    if ( digitalRead(versionGPIO) ) {
        whiteBoard = true;
    } else {
        greenBoard = true;
    }
    // pinMode(simpleLED, OUTPUT);
    // digitalWrite(simpleLED, HIGH);

    setupSymtable();  // symbols to be logged.
    setupDebug();
    setupNeoSupp();
    setupFlow();
    for (int i = 0; i < 5; i++) {
        colorSingleDot(i, 240);
    }
    colorSingleDot(1, 120.0);
    setupSerial();
    colorSingleDot(2, 120.0);
    if ( normalComm ) {
        setupComm();
    } else {
        //  We have a bypass mode to test the quadcopter hardware with the unmodified code.
        // This will only work if all of the directly connected SPI bus lines are
        // left as inputs. The slave bus to the cortex is always configured that way, so
        // we can use it, but we can't configure the other SPI bus.
        spiSlave_init();
    }
    setupRangeFinders();
    colorSingleDot(3, 120.0);
    setupThinking();
    colorSingleDot(4, 120.0);
    initTime = millis();
}

extern void baseLoop()
{
    while (1) {
        colorSingleDot(1, 60);
        debugStart = millis();
        pollDebug();
        colorSingleDot(1, 120);
        commStart = millis();

        debugTime = debugTime + commStart - debugStart;
        pollComm();
        colorSingleDot(1, 180);

        // if ( ! byPassManipulation ) {
        neoStart = millis();
        commTime = commTime + neoStart - commStart;
        pollNeoSupp();
        colorSingleDot(1, 240);
        rangeStart = millis();
        // neoTime = neoTime + rangeStart - neoStart;
        pollRangeFinders();
        colorSingleDot(1, 300);
        thinkStart = millis();
        rangeTime = rangeTime + thinkStart - rangeStart;
        pollFlow();
        pollThinking();
        colorSingleDot(1, 360);
        thinkTime = thinkTime + millis() - thinkStart;
        if ( micros() - lastHeartBeat > 50000 ) {

            if ( thinkStart % 666 > 222 ) {
                rgbSingleDot1(0, 1.0, 0.0, 0.0);
            } else {
                rgbSingleDot1(0, 0.0, 1.0, 0.0);
            }
        } else {
            colorSingleDot(0, iter++);
        }
    }
}

boolean waitForConnection = true;
void setupSerial() {
    Serial.begin(115200);
    if ( waitForConnection ) {
        for (int i = 4; i > 0; i--) {
            Serial.print(i);
            delay(1000);
        }
        if ( whiteBoard ) {
            Serial.println("whiteBoard");
        } else if ( greenBoard ) {
            Serial.println("greenBoard");
        } else {
            Serial.println("mystery board");
        }
    }
}
const boolean normalComm = true;

const int versionGPIO = 4; // grounded on greenboard.
long initTime = 0;
extern int activeRangeFinderCnt;

extern void pollWatcher();
long debugTime = 0, debugStart = 0;
long commTime = 0, commStart = 0;
long neoTime = 0, neoStart = 0;
long rangeTime = 0, rangeStart = 0;
long thinkTime = 0, thinkStart = 0;
long nextDebugPacket = 1000;
long iter = 0;