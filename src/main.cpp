#include "zdi.h"
#include "cpu.h"
#include "eZ80F92.h"
#include <esp_task_wdt.h>
#include "esp32_io.h"
#include "updater.h"
#include "message.h"
#include <CRC32.h>
#include <XModem.h>

#define PAGESIZE            2048
#define USERLOAD            0x40000 // start of RAM
#define BREAKPOINT          0x40020
#define FLASHLOAD           0x50000
#define MOSSIZE_ADDRESS     0x70000
#define MOSDONE             0x70003

#define LEDFLASHFASTMS      75
#define LEDFLASHSTEADYMS    500
#define LEDFLASHWAIT        300

#define WAITHOLDBUTTONMS    2000
#define WAITRESETMS         10000
#define WAITPROGRAMSECS     10
#define ZDI_TCKPIN 26
#define ZDI_TDIPIN 27

#define EZ80F92_FLASHSIZE (128*1024)
uint8_t* flash_buf;
size_t flash_buf_idx = 0;
unsigned long last_block_receive_time = 0;
bool received_one_block = false;
XModem xmodem;

int ledpins[] = {2,4,16}; // should cover most esp32 boards, including ezsbc board
int ledpins_onstate[] = {1,1,0}; // 1: Pin high == on, 0: Pin low == on

uint32_t expected_moscrc;
unsigned long time0;
bool btn_pressed;
bool needmenu;

CPU*                    cpu;
ZDI*                    zdi;

char zdi_msg_up[] = "ZDI up - id 0x%04X, revision 0x%02X\r\n";
char zdi_msg_down[] = "ZDI down - check cabling\r\n";
char menuHeader[] = "Agon flashing utility v1.0\r\n\r\n";

bool process_block(void *blk_id, size_t idSize, byte *data, size_t dataSize);

void setupLedPins(void) {
    for(int n = 0; n < (sizeof(ledpins) / sizeof(int)); n++)
        directModeOutput(ledpins[n]);
}

void ledsOff(void) {
    for(int n = 0; n < (sizeof(ledpins) / sizeof(int)); n++)
        if(ledpins_onstate[n]) directWriteLow(ledpins[n]);
        else directWriteHigh(ledpins[n]);
}

void ledsOn(void) {
    for(int n = 0; n < (sizeof(ledpins) / sizeof(int)); n++)
        if(ledpins_onstate[n]) directWriteHigh(ledpins[n]);
        else directWriteLow(ledpins[n]);
}

void ledsFlash(void) {
    ledsOff();
    delay(LEDFLASHFASTMS);
    ledsOn();
    delay(LEDFLASHFASTMS);
}

void ledsErrorFlash(void) {
    int n;
    for(n = 0; n < 4; n++) ledsFlash();
    delay(LEDFLASHWAIT);
}

void ledsWaitFlash(int32_t ms) {
    while(ms > 0) {
        ledsOff();
        delay(LEDFLASHSTEADYMS);
        ledsOn();
        delay(LEDFLASHSTEADYMS);
        ms -= 2*LEDFLASHSTEADYMS;
    }
}

uint32_t getDataCRC(const uint8_t* data, size_t len) {
    CRC32 crc;
    for(size_t n = 0; n < len; n++) 
        crc.update(data[n]);
    return crc.finalize();
}

uint32_t getfileCRC(const char *name) {
    CRC32 crc;
    uint32_t size, retsize;
    uint8_t buffer[PAGESIZE];

    FILE *file = fopen(name, "r");
    
    while(size = fread(buffer, 1, PAGESIZE, file)) {
        for(int n = 0; n < size; n++) crc.update(buffer[n]);
    }

    fclose(file);
    return crc.finalize();
}

void setup() {
    // Disable the watchdog timers
    disableCore0WDT(); delay(200);								
    esp_task_wdt_init(30, false); // in case WDT cannot be removed

    // Serial
    Serial.begin(115200);

    flash_buf = (uint8_t*) malloc(EZ80F92_FLASHSIZE);
    if(flash_buf == nullptr) {
        Serial.println("FAIL: COULD NOT ALLOCATE FLASH BUF");
    }

    // setup ZDI interface
    zdi = new ZDI(ZDI_TCKPIN, ZDI_TDIPIN);
    cpu = new CPU(zdi);

    // setup LED pins
    setupLedPins();

    // configure boot button as input
    directModeInput(0);

    // Start timer
    time0 = millis();
    btn_pressed = false;

    // Boot-up display
    needmenu = true;

    // Wait a little for serial comms
    delay(1000);

    //expected_moscrc = getfileCRC("/spiffs/MOS.bin");
    // initialize some stuff
    xmodem.begin(Serial, XModem::ProtocolType::XMODEM);

}


void init_ez80(void) {
    cpu->setBreak();
    cpu->setADLmode(true);
    cpu->instruction_di();  
    
    // configure SPI
    cpu->instruction_out (SPI_CTL, 0x04);
    // configure default GPIO
    cpu->instruction_out (PB_DDR, 0xff);
    cpu->instruction_out (PC_DDR, 0xff);
    cpu->instruction_out (PD_DDR, 0xff);
    
    cpu->instruction_out (PB_ALT1, 0x0);
    cpu->instruction_out (PC_ALT1, 0x0);
    cpu->instruction_out (PD_ALT1, 0x0);
    cpu->instruction_out (PB_ALT2, 0x0);
    cpu->instruction_out (PC_ALT2, 0x0);
    cpu->instruction_out (PD_ALT2, 0x0);

    cpu->instruction_out (TMR0_CTL, 0x0);
    cpu->instruction_out (TMR1_CTL, 0x0);
    cpu->instruction_out (TMR2_CTL, 0x0);
    cpu->instruction_out (TMR3_CTL, 0x0);
    cpu->instruction_out (TMR4_CTL, 0x0);
    cpu->instruction_out (TMR5_CTL, 0x0);

    cpu->instruction_out (UART0_IER, 0x0);
    cpu->instruction_out (UART1_IER, 0x0);

    cpu->instruction_out (I2C_CTL, 0x0);
    cpu->instruction_out (FLASH_IRQ, 0x0);

    cpu->instruction_out (SPI_CTL, 0x4);

    cpu->instruction_out (RTC_CTRL, 0x0);
    
    // configure internal flash
    cpu->instruction_out (FLASH_ADDR_U,0x00);
    cpu->instruction_out (FLASH_CTRL,0b00101000);   // flash enabled, 1 wait state
    
    // configure internal RAM chip-select range
    cpu->instruction_out (RAM_ADDR_U,0xb7);         // configure internal RAM chip-select range
    cpu->instruction_out (RAM_CTL,0b10000000);      // enable
    // configure external RAM chip-select range
    cpu->instruction_out (CS0_LBR,0x04);            // lower boundary
    cpu->instruction_out (CS0_UBR,0x0b);            // upper boundary
    cpu->instruction_out (CS0_BMC,0b00000001);      // 1 wait-state, ez80 mode
    cpu->instruction_out (CS0_CTL,0b00001000);      // memory chip select, cs0 enabled

    // configure external RAM chip-select range
    cpu->instruction_out (CS1_CTL,0x00);            // memory chip select, cs1 disabled
    // configure external RAM chip-select range
    cpu->instruction_out (CS2_CTL,0x00);            // memory chip select, cs2 disabled
    // configure external RAM chip-select range
    cpu->instruction_out (CS3_CTL,0x00);            // memory chip select, cs3 disabled

    // set stack pointer
    cpu->sp(0x0BFFFF);
    // set program counter
    cpu->pc(0x000000);
}

// Upload to ZDI memory from a buffer
void ZDI_upload(uint32_t address, const uint8_t *buffer, uint32_t size, bool report) {
    while(size > PAGESIZE) {
        zdi->write_memory(address, PAGESIZE, buffer);
        address += PAGESIZE;
        buffer += PAGESIZE;
        size -= PAGESIZE;
        ledsFlash();
        if(report) displayMessage(".");
    }
    zdi->write_memory(address, size, buffer);
    if(report) displayMessage(".");
    ledsFlash();
}

// Upload to ZDI memory from a file
/*uint32_t ZDI_upload(uint32_t address, const char *name, bool report) {
    uint32_t size, retsize;
    uint8_t buffer[PAGESIZE];

    FILE *file = fopen(name, "r");
    
    retsize = 0;
    while(size = fread(buffer, 1, PAGESIZE, file)) {
        zdi->write_memory(address, size, buffer);
        address += size;
        retsize += size;
        ledsFlash();
        if(report) displayMessage(".");
    }

    fclose(file);
    return retsize;
}*/

uint32_t getZDImemoryCRC(uint32_t address, uint32_t size) {
    CRC32 crc;
    uint32_t crcbytes = PAGESIZE;
    uint8_t buffer[PAGESIZE];
    crc.reset();

    cpu->setBreak();
    while(size) {
        if(size >= PAGESIZE) {
            zdi->read_memory(address, PAGESIZE, buffer);
            address += PAGESIZE;
            size -= PAGESIZE;
        }
        else {
            zdi->read_memory(address, size, buffer);
            crcbytes = size;
            address += size;
            size = 0;
        }
        for(int n = 0; n < crcbytes; n++) crc.update(buffer[n]);
    }
    return crc.finalize();
}

/* assembled code for src_flash/flash.s. Aka, flash.bin contents*/
static const uint8_t flasher_prog[]  = {
  0x31, 0xff, 0xff, /* LD sp, 0BFFFFh */
  0x0b, 0x3e, 0x00, /* LD A, 0h */
  0x32, 0x03, 0x00, 0x07, /* LD (ackdone), A */
  0x3e, 0xb6, /* LD A, B6h */
  0xed, 0x39, 0xf5,  /* OUT0 (F5), A*/
  0x3e, 0x49, /* LD A, 49h */
  0xed, 0x39, 0xf5,  /* OUT0 (F5), A*/
  0x3e, 0x00, /* LD      A, 0h   ; unprotect all pages */
  0xed, 0x39, 0xfa,  /* OUT0    (FAh), A*/
  0x3e, 0xb6, /* LD      A, B6h  ; unlock again */
  0xed, 0x39, 0xf5, /* OUT0    (F5h), A */
  0x3e, 0x49, /* LD      A, 49h */
  0xed, 0x39, 0xf5, /* OUT0    (F5h), A */
  0x3e, 0x5f, /* LD      A, 5Fh  ; Ceiling(18Mhz * 5,1us) = 95, or 0x5F */
  0xed, 0x39, 0xf9, /* OUT0    (F9h), A */
  0x3e, 0x01, /* LD      A, 01h ; mass erase flash */ 
  0xed, 0x39, 0xff, /*     OUT0    (FFh), A */
  0xed, 0x38, 0xff, /* erasewait: IN0     A, (FFh) */
  0xe6, 0x01, /* AND     A, 01h */
  0x20, 0xf9, 0x11, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x05, 0xed, 0x4b, 0x00, 0x00, 
  0x07, 0xed, 0xb0, 0x3e, 0xb6, 0xed, 0x39, 0xf5, 0x3e, 0x49, 0xed, 0x39, 0xf5, 0x3e, 0xff, 0xed, 
  0x39, 0xfa, 0x3e, 0x01, 0x32, 0x03, 0x00, 0x07, 0x18, 0xfe
};

void upload_flasher_into_ram() {
    cpu->setBreak();
    ZDI_upload(USERLOAD, flasher_prog, sizeof(flasher_prog), true);
    cpu->pc(USERLOAD);
    Serial.println("Flasher loaded into RAM, CPU halted.");
    /* program will now expect "data to flash" */
}

void upload_block_to_flash(const uint8_t* data, size_t size) {
    /* we upload the to be flashed data into RAM first, the flasher
        * program will transfer it from RAM to flash
    */
    uint32_t expected_crc = getDataCRC(data, size);
    cpu->setBreak();
    ZDI_upload(FLASHLOAD, data, size, true);
    zdi->write_memory_24bit(MOSSIZE_ADDRESS, size);
    cpu->pc(USERLOAD);
    cpu->setContinue(); // start flashloader, no feedback  

    // This is a CRITICAL wait and cannot be interrupted by ZDI
    for(int n = 0; n < WAITPROGRAMSECS; n++) { 
        ledsFlash();
        delay(1000);
        displayMessage(".");
    }
    displayMessage(" done - ");

    /* wait a bit */
    uint32_t now = millis();
    // max 20 seconds flashing time
    uint8_t success = false;
    while(millis() - now < 20000) {
        cpu->setBreak();
        // check ackdone
        uint8_t ack_done = 0;
        zdi->read_memory(MOSDONE, 1, &ack_done);
        // flasher says it's done?
        if(ack_done == 1) {
            Serial.println("ACK received!");
            success = true;
            break;
        } else {
            Serial.println("Flasher wasn't done yet");
            uint32_t len_rem = 0;
            zdi->read_memory(MOSSIZE_ADDRESS, 3, (uint8_t*)&len_rem);
            Serial.println("Remaining length: " + String(len_rem));
            Serial.println("Current PC: " + String(cpu->pc(), HEX));
            Serial.println("A: " + String(cpu->a(), HEX));
            Serial.println("DE: " + String(cpu->de(), HEX));
            Serial.println("HL: " + String(cpu->hl(), HEX));
            Serial.println("BC: " + String(cpu->bc(), HEX));
        }
        // not yet done: resume program
        cpu->setContinue();
        delay(1000);
    }

    if(!success) {
        Serial.println("TIMEOUT DURING FLASH!\n");
        // return;
    }
    Serial.println("Verifying CRC32");
    // final check
    if(expected_crc == getZDImemoryCRC(0, size)) {
        displayMessage("CRC32 OK\r\n");
    }
    else {
        displayError("CRC32 ERROR\r\n");
    }
}
#if 0
void flashMOS(void) {
    uint32_t size;

    init_ez80();

    // Upload the MOS payload to ZDI memory first
    displayMessage("\r\nUploading MOS...");
    ledsFlash();
    size = ZDI_upload(FLASHLOAD, "/spiffs/MOS.bin", true);
    displayMessage(" done - ");
    if(expected_moscrc == getZDImemoryCRC(FLASHLOAD, size)) {
        displayMessage("CRC32 OK\r\n");
    }
    else {
        displayError("CRC32 ERROR\r\n");
        return;
    }
    zdi->write_memory_24bit(MOSSIZE_ADDRESS, size);
    ledsFlash();
     
    // Upload the flash tool to ZDI memory next, so it can pick up the payload
    displayMessage("Uploading flash tool...");
    ledsFlash();
    ZDI_upload(USERLOAD, "/spiffs/flash.bin", true);
    ledsFlash();

    // Run the CPU from the flash tool address
    displayMessage("\r\nFlashing MOS...");
    cpu->pc(USERLOAD);
    cpu->setContinue(); // start flashloader, no feedback  

    // This is a CRITICAL wait and cannot be interrupted by ZDI
    for(int n = 0; n < WAITPROGRAMSECS; n++) { 
        ledsFlash();
        delay(1000);
        displayMessage(".");
    }
    displayMessage(" done - ");
    
    // Final check
    cpu->setBreak();
    if(expected_moscrc == getZDImemoryCRC(0, size)) {
        displayMessage("CRC32 OK\r\n");
    }
    else {
        displayError("CRC32 ERROR\r\n");
    }
}
#endif

void printSerialMenu(void) {
    Serial.printf("\r\n\r\n---------------------------------\r\n");
    Serial.printf(zdi_msg_up, zdi->get_productid(), zdi->get_revision());
    Serial.printf("---------------------------------\r\n");
    Serial.printf(menuHeader);
}

void printMenus(void) {
    printSerialMenu();
}

void zdiStatusMessage(void) {
    while((zdi->get_productid() != 7)) {
        if(!needmenu) {
            needmenu = true;
        }
        ledsErrorFlash();
        ledsOff();
        Serial.printf(zdi_msg_down);
        delay(500);
    }
    if(needmenu) {
        printMenus();
        needmenu = false;
    }
    ledsOn();
}

// Return ASCII key from serial or PS/2 keyboard
// or 0 when nothing is pressed
// Non-blocking
char getKey(void) {
    char key = 0;
    if(Serial.available()) key = Serial.read();
    return key;
}

bool process_block(void *blk_id, size_t idSize, byte *data, size_t dataSize) {
    byte id = *((byte *) blk_id);
    //just copy into RAM buffer for now
    last_block_receive_time = millis();
    received_one_block = true;
    if(flash_buf_idx + dataSize < EZ80F92_FLASHSIZE) {
        memcpy(&flash_buf[flash_buf_idx], data, dataSize);
        flash_buf_idx += dataSize;
        return true;
    } else {
        // can't hold that much data
        return false;
    }
}

void loop() {
    zdiStatusMessage();
    if(Serial.available() > 0) {
        char c = (char) Serial.read();
        // write command?
        if( c == 'w') {
            init_ez80();
            upload_flasher_into_ram();
            Serial.println("Send file to be uploaded to 0x00000 via XModem now!");
            Serial.flush();
            flash_buf_idx = 0;
            last_block_receive_time = millis();
            received_one_block = false;
            xmodem.setRecieveBlockHandler(process_block);
            // process X modem until we detect a timeout of 200ms
            // but at least wait until one block is received
            //while(true) {
            bool ok = xmodem.receive();
            Serial.println("RX OK: " + String(ok));
                //if(millis() - last_block_receive_time >= 2000 && received_one_block) {
                //    break;
                //}
                //if((millis() / 1000) % 10 == 9) {
                //    Serial.println("Still waiting for data");
            Serial.println("idx: " + String(flash_buf_idx));
            Serial.println("last receive: " + String(last_block_receive_time));
            Serial.println("one block received: " + String(received_one_block));
                //}
            //}
            // we don't use xmodem anymore, back to regular serial
            Serial.println("Received " + String(flash_buf_idx) + " Bytes to flash");
            upload_block_to_flash(flash_buf, flash_buf_idx);
            Serial.println("Upload done!");
        } else if(c == 'r') {
            // read command?
        }
    }
}
