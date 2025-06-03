#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Servo.h>

// Servo motor setup
Servo myServo;
const int SERVO_PIN = 8;  // Changed from D9 to D8 since D9 is used for LoRa reset
int servoOriginalPosition = 0;  // Starting position
bool servoActive = false;

// *** YOUR TTN APPLICATION CREDENTIALS ***
// AppEUI in little-endian format (reversed from: 60C5A8FFFE74D21D)
static const u1_t PROGMEM APPEUI[8] = { 0x1D, 0xD2, 0x74, 0xFE, 0xFF, 0xA8, 0xC5, 0x60 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DevEUI in little-endian format (reversed from: 70B3D57ED0071281)
static const u1_t PROGMEM DEVEUI[8] = { 0x81, 0x12, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// AppKey in big-endian format (as provided: 1AF13CF82985388347C0A329E12E6C44)
static const u1_t PROGMEM APPKEY[16] = { 0x1A, 0xF1, 0x3C, 0xF8, 0x29, 0x85, 0x38, 0x83, 0x47, 0xC0, 0xA3, 0x29, 0xE1, 0x2E, 0x6C, 0x44 };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// Pin mapping for RF210 UCA board - ADJUSTED FOR SERVO
const lmic_pinmap lmic_pins = {
    .nss = 10,      // Connected to pin 10 (CS/NSS)
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,       // Connected to pin 9 (Reset) - SERVO MOVED TO PIN 8
    .dio = {2, 6, LMIC_UNUSED_PIN}, // DIO0, DIO1, DIO2
};

// Application state
static osjob_t sendjob;
bool joined = false;
String lastMessage = "";

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting LoRaWAN TTN OTAA Receiver"));

    // LMIC init
    os_init();
    
    // Reset the MAC state
    LMIC_reset();

    // Set up the channels used by the Things Network
    // This is for EU868 - adjust for your region
    setupEU868Channels();
    
    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7, 14);

    // Start job (joining)
    do_join(&sendjob);
    
    // Initialize servo motor
    myServo.attach(SERVO_PIN);
    myServo.write(servoOriginalPosition);  // Set to starting position
    delay(500);  // Give servo time to reach position
    
    Serial.println(F("Servo motor initialized on pin 8"));
    Serial.println(F("Send 'servo' command to activate servo motion"));
    Serial.println(F("Attempting to join TTN via OTAA..."));
}

void loop() {
    os_runloop_once();
}

void setupEU868Channels() {
    // Set up the channels used in EU868
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    
    // Channel 0: 868.1 MHz / DR0-5
    // Channel 1: 868.3 MHz / DR0-5  
    // Channel 2: 868.5 MHz / DR0-5
    // Channel 3: 867.1 MHz / DR0-5
    // Channel 4: 867.3 MHz / DR0-5
    // Channel 5: 867.5 MHz / DR0-5
    // Channel 6: 867.7 MHz / DR0-5
    // Channel 7: 867.9 MHz / DR0-5
    
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
}

void do_join(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not joining"));
    } else {
        // Start joining
        LMIC_startJoining();
    }
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            joined = true;
            
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            
            Serial.println(F("Successfully joined TTN!"));
            Serial.println(F("Ready to receive downlink messages"));
            
            // Schedule first uplink to indicate we're ready
            scheduleHeartbeat();
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            Serial.println(F("Check your keys and try again..."));
            // Retry join after 60 seconds
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(60), do_join);
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            Serial.println("=== TRANSMISSION DETAILS ===");
            Serial.print(F("TX Power: "));
            Serial.println(LMIC.txpow);
            Serial.print(F("Data Rate: "));
            Serial.println(LMIC.datarate);
            Serial.print(F("Frequency: "));
            Serial.println(LMIC.freq);
            Serial.print(F("TX Channel: "));
            Serial.println(LMIC.txChnl);
            Serial.println("=============================");
            
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              
              // Process received downlink message
              processDownlinkMessage();
            }
            // Schedule next heartbeat
            scheduleHeartbeat();
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println(ev);
            break;
    }
}

void processDownlinkMessage() {
    Serial.println(F("=== DOWNLINK MESSAGE RECEIVED ==="));
    Serial.print(F("Length: "));
    Serial.println(LMIC.dataLen);
    Serial.print(F("Port: "));
    Serial.println(LMIC.frame[LMIC.dataBeg-1]);
    
    // Convert received data to string
    String message = "";
    for (int i = 0; i < LMIC.dataLen; i++) {
        message += (char)LMIC.frame[LMIC.dataBeg + i];
    }
    
    Serial.print(F("Message: "));
    Serial.println(message);
    Serial.print(F("RSSI: "));
    Serial.println(LMIC.rssi);
    Serial.print(F("SNR: "));
    Serial.println(LMIC.snr);
    
    // Store last message
    lastMessage = message;
    
    // Parse and act on the message
    parseAndExecuteCommand(message);
    
    Serial.println(F("================================"));
}

void parseAndExecuteCommand(String command) {
    command.trim();
    command.toLowerCase();
    
    Serial.println(F("--- Parsing Command ---"));
    Serial.println("Command: " + command);
    
    // LED control commands
    if (command.indexOf("led") != -1) {
        if (command.indexOf("on") != -1 || command.indexOf("1") != -1) {
            controlLED(true);
        } else if (command.indexOf("off") != -1 || command.indexOf("0") != -1) {
            controlLED(false);
        }
    }
    
    // Relay control commands
    if (command.indexOf("relay") != -1) {
        if (command.indexOf("on") != -1 || command.indexOf("1") != -1) {
            controlRelay(true);
        } else if (command.indexOf("off") != -1 || command.indexOf("0") != -1) {
            controlRelay(false);
        }
    }
    
    // Servo control commands
    if (command.indexOf("servo") != -1) {
        activateServo();
    }
    
    // Status request
    if (command.indexOf("status") != -1) {
        Serial.println(F("Status requested - will send in next uplink"));
    }
    
    // Reset command
    if (command.indexOf("reset") != -1) {
        Serial.println(F("Reset command received"));
        delay(1000);
        // Uncomment next line if you want to enable reset functionality
        // asm volatile ("  jmp 0");
    }
}

void controlLED(bool state) {
    const int LED_PIN = LED_BUILTIN;
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, state ? HIGH : LOW);
    
    Serial.print(F("LED turned "));
    Serial.println(state ? F("ON") : F("OFF"));
}

void controlRelay(bool state) {
    const int RELAY_PIN = 7; // Adjust pin as needed
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, state ? HIGH : LOW);
    
    Serial.print(F("Relay turned "));
    Serial.println(state ? F("ON") : F("OFF"));
}

void activateServo() {
    if (servoActive) {
        Serial.println(F("Servo already active, ignoring command"));
        return;
    }
    
    servoActive = true;
    Serial.println(F("=== SERVO ACTIVATION ==="));
    Serial.println(F("Starting servo motion sequence..."));
    
    // Move servo to 90 degrees
    Serial.println(F("Moving servo to 90 degrees"));
    myServo.write(90);
    delay(1500);  // Wait for servo to reach position (1.5 seconds)
    
    Serial.println(F("Servo reached 90 degrees"));
    delay(1000);  // Hold position for 1 second
    
    // Return servo to original position
    Serial.println(F("Returning servo to original position (0 degrees)"));
    myServo.write(servoOriginalPosition);
    delay(1500);  // Wait for servo to return
    
    Serial.println(F("Servo returned to original position"));
    Serial.println(F("Servo motion sequence complete"));
    Serial.println(F("========================"));
    
    servoActive = false;
}

void scheduleHeartbeat() {
    // Schedule next transmission (heartbeat every 30 seconds for debugging)
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(30), do_send);
}

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Try different payload formats to see what works
        
        // Test 1: Simple string
        String testPayload = "Hello TTN";
        Serial.println("=== SENDING UPLINK ===");
        Serial.println("Payload: " + testPayload);
        Serial.println("Length: " + String(testPayload.length()));
        Serial.println("Port: 1");
        
        // Print payload as hex for debugging
        Serial.print("Hex: ");
        for (int i = 0; i < testPayload.length(); i++) {
            if (testPayload[i] < 0x10) Serial.print("0");
            Serial.print(testPayload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        LMIC_setTxData2(1, (uint8_t*)testPayload.c_str(), testPayload.length(), 0);
        Serial.println(F("Payload queued for transmission"));
        Serial.println("======================");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// Utility functions
void printHex(const uint8_t* data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void printKeys() {
    Serial.println(F("=== LoRaWAN Keys (for debugging) ==="));
    Serial.print(F("APPEUI: "));
    for (int i = 7; i >= 0; i--) {
        if (APPEUI[i] < 0x10) Serial.print("0");
        Serial.print(APPEUI[i], HEX);
    }
    Serial.println();
    
    Serial.print(F("DEVEUI: "));
    for (int i = 7; i >= 0; i--) {
        if (DEVEUI[i] < 0x10) Serial.print("0");
        Serial.print(DEVEUI[i], HEX);
    }
    Serial.println();
    Serial.println(F("APPKEY: [hidden for security]"));
    Serial.println(F("==================================="));
}