//  SEGA SC-3000 emulator
//
//  GP0: HSYNC
//  GP1: VSYNC
//  GP2: Blue0
//  GP3: Blue1
//  GP4: Red0
//  GP5: Red1
//  GP6: Red2
//  GP7: Green0
//  GP8: Green1
//  GP9: Green2
//  GP10: Audio

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "hardware/vreg.h"

#include "tusb.h"
#include "bsp/board.h"
#include "hidparser/hidparser.h"

#include "vga16_graphics.h"
#include "tms9918/vrEmuTms9918.h"
#include "tms9918/vrEmuTms9918Util.h"

#include "sc3000keymap.h"
#include "Z80.h"
#include "msxmisc.h"
#include "font_jp.h"

#include "lfs.h"

// VGAout configuration

#define DOTCLOCK 25000
#define CLOCKMUL 9

// Pico does not work at CLOCKMUL=7 (175MHz).

#define VGA_PIXELS_X 320
#define VGA_PIXELS_Y 200

#define VGA_CHARS_X 40
#define VGA_CHARS_Y 24

#define VRAM_PAGE_SIZE (VGA_PIXELS_X*VGA_PIXELS_Y/8)

extern unsigned char vga_data_array[];
volatile uint8_t fbcolor,cursor_x,cursor_y,video_mode;

volatile uint32_t video_hsync,video_vsync,scanline,vsync_scanline;

struct repeating_timer timer,timer2;

// PC configuration

static Z80 cpu;
uint32_t cpu_clocks=0;
uint32_t cpu_ei=0;
uint32_t cpu_cycles=0;
uint32_t cpu_hsync=0;

uint32_t cpu_trace=0;   // DEBUG
uint32_t cpu_boost=0;

uint8_t megarom[8];

uint8_t mainram[0x10000];
uint8_t ioport[0x100];

uint32_t cart_loaded[2]={0,0};
uint32_t enable_nmi;

// VDP
VrEmuTms9918 *mainscreen,*menuscreen;
uint8_t scandata[256];

uint8_t timer_enable_irq=0;

volatile uint8_t redraw_flag=0;

uint16_t keymap[8];

volatile uint8_t keypressed=0;  //last pressed usbkeycode

// BEEP & PSG

uint32_t beep_enable=0;
uint32_t pwm_slice_num;
volatile uint32_t sound_tick=0;

#define PSG_NUMBERS 1

uint16_t psg_register[8 * PSG_NUMBERS];
uint32_t psg_osc_interval[3 * PSG_NUMBERS];
uint32_t psg_osc_counter[3 * PSG_NUMBERS];

uint32_t psg_noise_interval[PSG_NUMBERS];
uint32_t psg_noise_counter[PSG_NUMBERS];
uint8_t psg_noise_output[PSG_NUMBERS];
uint32_t psg_noise_seed[PSG_NUMBERS];
uint32_t psg_freq_write[PSG_NUMBERS];
// uint32_t psg_envelope_interval[PSG_NUMBERS];
// uint32_t psg_envelope_counter[PSG_NUMBERS];
//uint32_t psg_master_clock = PSG_CLOCK2;
//uint32_t psg_master_clock = (3579545/2);
uint32_t psg_master_clock = 3579545;
uint16_t psg_master_volume = 0;

// TESUTO
uint32_t psg_note_count;

//const uint16_t psg_volume[] = { 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04,
//        0x05, 0x06, 0x07, 0x08, 0x09, 0x0b, 0x0d, 0x10, 0x13, 0x17, 0x1b, 0x20,
//        0x26, 0x2d, 0x36, 0x40, 0x4c, 0x5a, 0x6b, 0x80, 0x98, 0xb4, 0xd6, 0xff };

const uint16_t psg_volume[] = { 0xFF,0xCB,0xA1,0x80,0x66,0x51,0x40,0x33,0x28,0x20,0x1A,0x14,0x10,0x0D,0x0A,0x00};

#define SAMPLING_FREQ 22050

#define TIME_UNIT 100000000                           // Oscillator calculation resolution = 10nsec
#define SAMPLING_INTERVAL (TIME_UNIT/SAMPLING_FREQ) 

// Tape

uint32_t tape_ready=0;
uint32_t tape_ptr=0;
uint32_t tape_phase=0;
uint32_t tape_count=0;

uint32_t tape_read_wait=0;
uint32_t tape_leader=0;
uint32_t tape_autoclose=1;          // Default value of TAPE autoclose
uint32_t tape_skip=0;               // Default value of TAPE load accelaration
uint32_t tape_cycles;
uint8_t tape_suspend;

const uint32_t tape_waits[] = { 750 ,1500 , 750 , 1500 } ;  // Tape signal width

#define TAPE_WAIT_WIDTH 40

#define TAPE_THRESHOLD 20000000

uint8_t uart_rx[32];
uint8_t uart_nibble=0;
uint8_t uart_count=0;
volatile uint8_t uart_write_ptr=0;
volatile uint8_t uart_read_ptr=0;
uint32_t uart_cycle;

// UI

uint32_t menumode=0;
uint32_t menuitem=0;

// USB

hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
extern void hid_app_task(void);
extern volatile uint16_t gamepad_info;
uint32_t gamepad_select=0;

uint32_t usbcheck_count=0;
uint32_t kbhit=0;            // 4:Key pressed (timer stop)/3&2:Key depressed (timer running)/1:no key irq triggerd
uint8_t hid_dev_addr=255;
uint8_t hid_instance=255;
uint8_t hid_led;

#define USB_CHECK_INTERVAL 30 // 31.5us*30=1ms

// Define the flash sizes
// This is setup to read a block of the flash from the end 
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)
// for 1M flash pico
//#define HW_FLASH_STORAGE_BASE   (1024*1024 - HW_FLASH_STORAGE_BYTES) 
//#define HW_FLASH_STORAGE_BYTES  (512 * 1024)
// for 2M flash
// #define HW_FLASH_STORAGE_BYTES  (1024 * 1024)
#define HW_FLASH_STORAGE_BYTES  (1536 * 1024)
#define HW_FLASH_STORAGE_BASE   (PICO_FLASH_SIZE_BYTES - HW_FLASH_STORAGE_BYTES) 
// for 16M flash
//#define HW_FLASH_STORAGE_BYTES  (15872 * 1024)
//#define HW_FLASH_STORAGE_BASE   (1024*1024*16 - HW_FLASH_STORAGE_BYTES) 

uint8_t __attribute__  ((aligned(sizeof(unsigned char *)*4096))) flash_buffer[4096];

lfs_t lfs;
lfs_file_t lfs_file,lfs_fd,lfs_cart1,lfs_cart2;

#define FILE_THREHSOLD 20000000
#define LFS_LS_FILES 15

volatile uint32_t load_enabled=0;
volatile uint32_t save_enabled=0;
//uint32_t file_cycle=0;

unsigned char filename[16];
unsigned char tape_filename[16];
unsigned char fd_filename[16];
unsigned char cart1_filename[16];
unsigned char cart2_filename[16];

static inline unsigned char tohex(int);
static inline unsigned char fromhex(int);
static inline void video_print(uint8_t *);

// *REAL* H-Sync for emulation
void __not_in_flash_func(hsync_handler)(void) {

    uint32_t vramindex;
    uint32_t tmsscan;
    uint8_t bgcolor;

    pio_interrupt_clear(pio0, 0);

    if((scanline!=0)&&(gpio_get(1)==0)) { // VSYNC
        scanline=0;
        video_vsync=1;
    } else {
        scanline++;
    }

    if((scanline%2)==0) {
        video_hsync=1;

        // VDP Draw on HSYNC

        // VGA Active starts scanline 35
        // TMS9918 Active scanline 75(0) to 474(199)

        if(scanline==78) {
            if(menumode==0) {
                bgcolor=vrEmuTms9918RegValue(mainscreen,TMS_REG_FG_BG_COLOR) & 0x0f;
            } else {
                bgcolor=vrEmuTms9918RegValue(menuscreen,TMS_REG_FG_BG_COLOR) & 0x0f;
            }
            memset(vga_data_array+320*4,colors[bgcolor],320);
        }

//        if((scanline>=75)&&(scanline<=456)) {
        if((scanline>=81)&&(scanline<=464)) {

            tmsscan=(scanline-81)/2;
            if(menumode==0) {
                vrEmuTms9918ScanLine(mainscreen,tmsscan,scandata);
                bgcolor=vrEmuTms9918RegValue(mainscreen,TMS_REG_FG_BG_COLOR) & 0x0f;
            } else {
                vrEmuTms9918ScanLine(menuscreen,tmsscan,scandata);
                bgcolor=vrEmuTms9918RegValue(menuscreen,TMS_REG_FG_BG_COLOR) & 0x0f;
            }
            vramindex=(tmsscan%4)*320;

            memset(vga_data_array+(tmsscan%4)*320,colors[bgcolor],32);
            memset(vga_data_array+(tmsscan%4)*320+32+256,colors[bgcolor],32);

            for(int j=0;j<256;j++) {
                vga_data_array[vramindex+j+32]=colors[scandata[j]];
            }           
        }

    }

    return;

}

// BEEP and PSG emulation
bool __not_in_flash_func(sound_handler)(struct repeating_timer *t) {

    uint16_t timer_diffs;
    uint32_t pon_count;
    uint16_t master_volume;
    uint32_t beep_on,beep_volume;
    uint8_t tone_output[3 * PSG_NUMBERS], noise_output[3 * PSG_NUMBERS];

    pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,psg_master_volume);

    // PSG

    // Run Noise generator

    for (int i = 0; i < PSG_NUMBERS; i++) {

        psg_noise_counter[i] += SAMPLING_INTERVAL;
        if (psg_noise_counter[i] > psg_noise_interval[i]) {
            psg_noise_seed[i] = (psg_noise_seed[i] >> 1)
                    | (((psg_noise_seed[i] << 14) ^ (psg_noise_seed[i] << 16))
                            & 0x10000);
            psg_noise_output[i] = psg_noise_seed[i] & 1;
            psg_noise_counter[i] -= psg_noise_interval[i];
        }

    }

    // Run Oscillator

    for (int i = 0; i < 3 * PSG_NUMBERS; i++) {
        pon_count = psg_osc_counter[i] += SAMPLING_INTERVAL;
        if (pon_count < (psg_osc_interval[i] / 2)) {
//            tone_output[i] = psg_tone_on[i];
            tone_output[i] = 1;
        } else if (pon_count > psg_osc_interval[i]) {
            psg_osc_counter[i] -= psg_osc_interval[i];
//            tone_output[i] = psg_tone_on[i];
            tone_output[i] = 1;
        } else {
            tone_output[i] = 0;
        }
    }

    // Mixer

    master_volume = 0;
    psg_note_count=0;

    for (int i = 0; i < PSG_NUMBERS; i++) {
        for (int j = 0; j < 3; j++) {
            if(tone_output[j+i*3]) {
                master_volume+=psg_volume[psg_register[j*2+i*8+1]];
//                master_volume+=psg_volume[32];
            } 
        }
        if(psg_noise_output[i]) {
            master_volume+=psg_volume[psg_register[7+i*8]];
        }
    }

    // count enable channels

    for (int i = 0; i < PSG_NUMBERS; i++) {
        for (int j = 0; j < 4; j++) {
            if(psg_register[j*2+i*8+1]!=0xf) {
                    psg_note_count++;
            }            
        }
    }

//    psg_master_volume = master_volume / (3 * PSG_NUMBERS);
    psg_master_volume = master_volume / psg_note_count;

    return true;
}

// PSG virtual registers
// 0: CH0 Freq
// 1: CH0 Volume
// 6: Noise Freq
// 7: Noise Volume

//void psg_write(uint32_t psg_no,uint32_t data) {
void psg_write(uint32_t data) {

    const uint8_t psg_no=0;
    uint32_t channel,freqdiv,freq;

    if(data&0x80) {

        channel=(data&0x60)>>5;
        psg_freq_write[psg_no]=0;

        switch((data&0x70)>>4) {

            // Frequency

            case 0:
            case 2:
            case 4:

                psg_register[psg_no*8+channel*2]=data&0xf;
                psg_freq_write[psg_no]=channel;
                break;

            case 6:  // WIP
                psg_register[psg_no*8+6]=data&0xf;
                switch(data&3){
                    case 0:
                        freqdiv=512;
                        break;
                    case 1:
                        freqdiv=1024;
                        break;
                    case 2:
                        freqdiv=2048;
                        break;
                    case 3:
                        freqdiv=psg_register[psg_no*8+4];
                }


                if(freqdiv==0) {
                    psg_noise_interval[psg_no]=UINT32_MAX;
                    return;
                }

                freq= psg_master_clock / freqdiv;
                freq>>=5;

                if(freq==0) {
                    psg_noise_interval[psg_no]=UINT32_MAX; 
                } else {
                    psg_noise_interval[psg_no]= TIME_UNIT/freq;
                    psg_noise_counter[psg_no]=0;
                }

                break;

            // volume

            case 1:
            case 3:
            case 5:
            case 7:
            
                psg_register[psg_no*8+channel*2+1]=data&0xf;

                break;

        }

    } else {

        uint32_t noise_flag=psg_register[psg_no*8+6]&3;
        
        channel=psg_freq_write[psg_no];
        psg_register[psg_no*8+channel*2]|=(data&0x3f)<<4;

        freqdiv=psg_register[psg_no*8+channel*2];

        if(freqdiv==0) {
            psg_osc_interval[psg_no*3+channel]=UINT32_MAX;
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=UINT32_MAX;
            }
            return;
        }

        freq= psg_master_clock / freqdiv;
        freq>>=5;

        if(freq==0) {
            psg_osc_interval[psg_no*3+channel]=UINT32_MAX; 
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=UINT32_MAX;
            }
        } else {
            psg_osc_interval[psg_no*3+channel]= TIME_UNIT/freq;
            psg_osc_counter[psg_no*3+channel]=0;
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=TIME_UNIT/freq;
                psg_noise_counter[psg_no]=0;
            }

        }

    }    
}

void __not_in_flash_func(uart_handler)(void) {

    uint8_t ch;

    if(uart_is_readable(uart0)) {
        ch=uart_getc(uart0);
        if(uart_count==0) {
            uart_nibble=fromhex(ch)<<4;
            uart_count++;
        } else {
            ch=fromhex(ch)+uart_nibble;
            uart_count=0;

            if(uart_read_ptr==uart_write_ptr+1) {  // buffer full
                return;
            }
            if((uart_read_ptr==0)&&(uart_write_ptr==31)) {
                return;
            }

            uart_rx[uart_write_ptr]=ch;
            uart_write_ptr++;
            if(uart_write_ptr>31) {
                uart_write_ptr=0;
            }
        }
    }

}

uint8_t tapein() {

static uint32_t tape_diff_cycles;
static uint8_t tape_bits,tape_file_data;
static uint8_t tape_half_bit,tape_signal;
static uint16_t tape_data;
static uint16_t tape_byte;
static uint32_t tape_header_bits;
static uint8_t tape_baud;
static uint8_t tape_last_bits;
static int32_t tape_geta_cycles;

    // if(tape_ready==0) {
    //     return 0;
    // }

    if(load_enabled==0) {
        return 0;
    }

    load_enabled=2;

    tape_diff_cycles=cpu_cycles-tape_cycles;
//    tape_cycles=cpu_cycles;
//    tape_last_bits=data;

    if(tape_phase%2) {

        if(tape_diff_cycles<(tape_waits[(tape_last_bits+1)%2]-tape_geta_cycles)) {
//        printf("[[%d]]",tape_diff_cycles);
            return tape_signal;
        }

//    printf("[D:%d,%d,%d,%d,%x]",tape_diff_cycles,tape_signal,tape_last_bits,tape_bits,tape_file_data);

        tape_cycles=cpu_cycles;

        if(tape_bits==0) { // start bit
            if(tape_signal==0) {   // 1 -> 0
                tape_signal=1;
                tape_bits++;
                tape_half_bit=0;
                tape_last_bits=(tape_file_data&1);
                return 1;
            } else {            // 0 -> 1
                tape_signal=0;
                return 0;
            }
        }
        if(tape_bits<9) {
            if(tape_signal==0) {   // 1 -> 0
                tape_geta_cycles=0;
                tape_signal=1;
                if(tape_last_bits) {
                    if(tape_half_bit==0) {
                        tape_half_bit++;
                        return 1;
                    }
                }
                if(tape_bits<8) {
                    tape_last_bits=tape_file_data>>tape_bits;
                    tape_last_bits&=1;
                    tape_bits++;
                    tape_half_bit=0;
                    return 1;
                } else {
                    tape_last_bits=1;
                    tape_bits++;
                    tape_half_bit=0;
                    return 1;                    
                }
                return 1;
            } else {            // 0 -> 1
                if(tape_diff_cycles>(2*tape_waits[0])) {  // Skip 
                    if(tape_last_bits) {
                        tape_half_bit++;
                    }
                }
                if((tape_diff_cycles<1870)&&(tape_diff_cycles>1830)) { // Need geta
                    if(tape_last_bits) {
                        tape_geta_cycles=390;
                        tape_signal=1;
                        return 1;
                    } else {
                        tape_geta_cycles=360;
                    }
                } else {
                    tape_geta_cycles=0;
                }
                tape_signal=0;
                return 0;
            }
        }

            if(tape_signal==0) {   // 1 -> 0

                if(tape_last_bits) {
                    if(tape_half_bit==0) {
                        tape_half_bit++;
                        tape_signal=1;
                        return 1;
                    }
                }
                if(tape_bits==9) {
                    tape_last_bits=1;
                    tape_bits++;
                    tape_signal=1;
                    tape_half_bit=0;
                    return 1;
                } else {
                    tape_last_bits=0;
                    tape_bits=0;
                    tape_signal=1;
                    tape_half_bit=0;
                    if(tape_ptr==22) {  // header section end
                        tape_phase++;
                        // 1sec blank
                        tape_suspend=1;
                        return 1;
                    }
                    lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);
//                    printf("[%x:%x]",tape_ptr,tape_file_data);
                    tape_data=tape_file_data;
                    tape_ptr++;

                    return 1;                    
                }
            } else {            // 0 -> 1
                tape_signal=0;
                return 0;
            }
        
    } else {
        // Header 
        // Return '1' 2400baud

        if((tape_suspend)&&(tape_signal==0)) { // MUTE
            if(tape_diff_cycles<10000000L) {
                return 0;
            } else {
                tape_suspend=0;
            }
        }

        if((tape_diff_cycles)>10000L) { // First 'h'
            tape_suspend=0;
            tape_cycles=cpu_cycles;
            tape_signal=0;
            tape_header_bits=0;
            return 0;
        }
        if(tape_diff_cycles>tape_waits[0]) {

//    printf("[H:%d,%d,%d,%d]",tape_diff_cycles,tape_signal,tape_header_bits,tape_phase);

            tape_cycles=cpu_cycles;
            if(tape_signal==0) {
                tape_signal=1;
                tape_header_bits++;
                if(tape_header_bits>7200) {
                    tape_bits=0;
                    tape_ptr++;
                    tape_last_bits=0;
                    tape_phase++;
                    tape_header_bits=0;
                    lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);
//                    printf("[%x:%x]",tape_ptr,tape_file_data);
                }
                return 1;
            } else {
                tape_signal=0;
                return 0;
            }
        } else {
            return tape_signal;
        }
    }

    return 0;

}

void tapeout(uint8_t data) {

static uint32_t tape_diff_cycles;
static uint8_t tape_bits,tape_file_data;
static uint8_t tape_half_bit;
static uint16_t tape_data;
static uint16_t tape_byte;
static uint8_t tape_baud;
static uint8_t tape_last_bits;

//    if(tape_ready) {

        if(tape_last_bits!=data) {

            tape_diff_cycles=cpu_cycles-tape_cycles;
            tape_cycles=cpu_cycles;
            tape_last_bits=data;

//            printf("[%d:%d]",data,tape_diff_cycles);

            if(tape_diff_cycles>10000L) {  // It is first H bit
                tape_baud=0;
                tape_bits=0;
                tape_byte=0;
                tape_phase=0;
            }

            // Skip headers

            if(tape_phase%2) {
                if(data==1) {
//                    printf("[%d:%d]",tape_bits,tape_diff_cycles);
                    if((tape_diff_cycles>tape_waits[1]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[1]+TAPE_WAIT_WIDTH)) {
                        tape_data=0;
                        tape_half_bit=0; 
//                            printf("0");
                    } else if((tape_diff_cycles>tape_waits[0]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[0]+TAPE_WAIT_WIDTH)) {
                        if(tape_half_bit) {
                            tape_data=0x8000;
                            tape_half_bit=0;
//                            printf("1");
                        } else {
                            tape_half_bit=1;
                            return;
                        }
                    } else if(tape_diff_cycles>10000L) {  // It's next bit
                            tape_data=0x8000;
                            tape_half_bit=0;
//                            printf("1");
                            tape_phase++;
                    }
                    tape_byte=(tape_byte>>1)|tape_data;
                    tape_bits++;
                    if(tape_bits==11) {
                        if(save_enabled) {
                            save_enabled=2;
                            tape_file_data=(tape_byte>>6)&0xff;
                            tape_ptr++;
                            lfs_file_write(&lfs,&lfs_file,&tape_file_data,1);
                        } else {
                            printf("[%02x]",(tape_byte>>6)&0xff);
                        }
                        tape_bits=0;
                        tape_byte=0;
                    }
                }                
            } else {

                if(data==1) {
                    if((tape_diff_cycles>tape_waits[1]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[1]+TAPE_WAIT_WIDTH)) {
                        // first '0' bit = Statbit of Data section
                        tape_bits=1;
                        tape_byte=0;
                        tape_phase++;
                    }
                }
            }
        }

//    } 
    
}

void menuinit(void) {

    // Initialize VDP

    vrEmuTms9918WriteRegisterValue(menuscreen, TMS_REG_0, 0b00000000);
    vrEmuTms9918WriteRegisterValue(menuscreen, TMS_REG_1, 0b01110000);
    vrEmuTms9918WriteRegisterValue(menuscreen, TMS_REG_2, 0b00001000);
    vrEmuTms9918WriteRegisterValue(menuscreen, TMS_REG_3, 0b00001000);
 
    vrEmuTms9918SetNameTableAddr(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS);
    vrEmuTms9918SetPatternTableAddr(menuscreen, TMS_DEFAULT_VRAM_PATT_ADDRESS);
    vrEmuTms9918SetFgBgColor(menuscreen, TMS_WHITE,TMS_BLACK);
    // Copy CG rom

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_PATT_ADDRESS);
    vrEmuTms9918WriteBytes(menuscreen, font, sizeof(font));

    // Clear screen 

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS);
    vrEmuTms9918WriteByteRpt(menuscreen,0,960);

    return;

}

static inline void video_cls() {
    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS);
    vrEmuTms9918WriteByteRpt(menuscreen,0,960);
}

static inline void video_print(uint8_t *string) {

    int len;
    uint8_t fdata;
    uint32_t vramindex;

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS + cursor_x + cursor_y*VGA_CHARS_X);

    len = strlen(string);

    for (int i = 0; i < len; i++) {

        vrEmuTms9918WriteData(menuscreen,string[i]);

        cursor_x++;
        if (cursor_x >= VGA_CHARS_X) {
            cursor_x = 0;
            cursor_y++;
            if (cursor_y >= VGA_CHARS_Y) {
//                video_scroll();
                cursor_y = VGA_CHARS_Y - 1;
            }
        }
    }

}

void draw_menu(void) {

    cursor_x=2;
    cursor_y=2;
    fbcolor=7;
      video_print("                                    ");
    for(int i=3;i<19;i++) {
        cursor_x=2;
        cursor_y=i;
        video_print("                                    ");
    }

    cursor_x=2;
    cursor_y=19;
    fbcolor=7;
    video_print("                                    ");

}

int draw_files(int num_selected,int page) {

    lfs_dir_t lfs_dirs;
    struct lfs_info lfs_dir_info;
    uint32_t num_entry=0;
    unsigned char str[16];

    int err= lfs_dir_open(&lfs,&lfs_dirs,"/");

    if(err) return -1;

    for(int i=0;i<LFS_LS_FILES;i++) {
        cursor_x=20;
        cursor_y=i+3;
        fbcolor=7;
//        video_print("                    ");
                video_print("  ");
    }

    while(1) {

        int res= lfs_dir_read(&lfs,&lfs_dirs,&lfs_dir_info);
        if(res<=0) {
            
            if(num_entry>=LFS_LS_FILES*(page+1)) {
                break;
            }

            if((num_entry%LFS_LS_FILES)!=(LFS_LS_FILES-1)) {
                for(int i=num_entry%LFS_LS_FILES;i<LFS_LS_FILES;i++) {
                    cursor_x=22;
                    cursor_y=i+3;
                    fbcolor=7;
                    video_print("                  ");                    
                }
            }

            break;
        }

        cursor_x=28;
        cursor_y=23;
        fbcolor=7;
        sprintf(str,"Page %02d",page+1);

        video_print(str);

        switch(lfs_dir_info.type) {

            case LFS_TYPE_DIR:
                break;
            
            case LFS_TYPE_REG:

                if((num_entry>=LFS_LS_FILES*page)&&(num_entry<LFS_LS_FILES*(page+1))) {

                    cursor_x=23;
                    cursor_y=num_entry%LFS_LS_FILES+3;

                    if(num_entry==num_selected) {
                        memcpy(filename,lfs_dir_info.name,16);
                    } else {
                        fbcolor=7;
                    }

                    snprintf(str,16,"%s            ",lfs_dir_info.name);
                    video_print(str);
//                    video_print(lfs_dir_info.name);

                }

                if(num_selected>=0) {
                    cursor_x=20;
                    cursor_y=(num_selected%LFS_LS_FILES)+3;
                    video_print("->");
                }

                num_entry++;

                break;

            default:
                break; 

        }

    }

    lfs_dir_close(&lfs,&lfs_dirs);

    return num_entry;

}

int file_selector(void) {

    uint32_t num_selected=0;
    uint32_t num_files=0;
    uint32_t num_pages=0;

    num_files=draw_files(-1,0);

    if(num_files==0) {
         return -1;
    }

    while(1) {

        while(video_vsync==0) ;
        video_vsync=0;

        draw_files(num_selected,num_selected/LFS_LS_FILES);

        tuh_task();

        if(keypressed==0x52) { // up
            keypressed=0;
            if(num_selected>0) {
                num_selected--;
            }
        }

        if(keypressed==0x51) { // down
            keypressed=0;
            if(num_selected<num_files-1) {
                num_selected++;
            }
        }

        if(keypressed==0x4b) { // Pageup
            keypressed=0;
            if(num_selected>=LFS_LS_FILES) {
                num_selected-=LFS_LS_FILES;
            }
        }

        if(keypressed==0x4e) { // Pagedown
            keypressed=0;
            if(num_selected<num_files-LFS_LS_FILES) {
                num_selected+=LFS_LS_FILES;
            }
        }

        if(keypressed==0x28) { // Ret
            keypressed=0;

            return 0;
        }

        if(keypressed==0x29 ) {  // ESC

            return -1;

        }

    }
}

int enter_filename() {

    unsigned char new_filename[16];
    unsigned char str[32];
    uint8_t keycode;
    uint32_t pos=0;

    memset(new_filename,0,16);

    while(1) {

        sprintf(str,"Filename:%s  ",new_filename);
        cursor_x=3;
        cursor_y=18;
        video_print(str);

        while(video_vsync==0) ;
        video_vsync=0;

        tuh_task();

        if(keypressed!=0) {

            if(keypressed==0x28) { // enter
                keypressed=0;
                if(pos!=0) {
                    memcpy(filename,new_filename,16);
                    return 0;
                } else {
                    return -1;
                }
            }

            if(keypressed==0x29) { // escape
                keypressed=0;
                return -1;
            }

            if(keypressed==0x2a) { // backspace
                keypressed=0;

                cursor_x=3;
                cursor_y=18;
                video_print("Filename:          ");

                new_filename[pos]=0;

                if(pos>0) {
                    pos--;
                }
            }

            if(keypressed<0x4f) {
                keycode=usbhidcode[keypressed*2];
                keypressed=0;

                if(pos<7) {

                    if((keycode>0x20)&&(keycode<0x5f)&&(keycode!=0x2f)) {

                        new_filename[pos]=keycode;
                        pos++;

                    }

                }
            }


        }
    }

}

//----------------------------------------------------------------------------------------------

void psg_reset(int flag) {

    psg_noise_seed[0] = 12345;

    for (int i = 0; i < 8; i+=2) {
        psg_register[i] = 0;
        psg_register[i+1] = 0xf;
    }

    psg_noise_interval[0] = UINT32_MAX;

    for (int i = 0; i < 3; i++) {
        psg_osc_interval[i] = UINT32_MAX;
//        psg_tone_on[i] = 0;
//        psg_noise_on[i] = 0;
    }

}

//----------------------------------------------------------------------------------------------------

static inline unsigned char tohex(int b) {

    if(b==0) {
        return '0';
    } 
    if(b<10) {
        return b+'1'-1;
    }
    if(b<16) {
        return b+'a'-10;
    }

    return -1;

}

static inline unsigned char fromhex(int b) {

    if(b=='0') {
        return 0;
    } 
    if((b>='1')&&(b<='9')) {
        return b-'1'+1;
    }
    if((b>='a')&&(b<='f')) {
        return b-'a'+10;
    }

    return -1;

}

// LittleFS

int pico_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t fs_start = XIP_BASE + HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] READ: %p, %d\n", addr, size);
    
    memcpy(buffer, (unsigned char *)addr, size);
    return 0;
}

int pico_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] WRITE: %p, %d\n", addr, size);
        
    uint32_t ints = save_and_disable_interrupts();
    multicore_lockout_start_blocking();     // pause another core
    flash_range_program(addr, (const uint8_t *)buffer, size);
    multicore_lockout_end_blocking();
    restore_interrupts(ints);
        
    return 0;
}

int pico_erase(const struct lfs_config *c, lfs_block_t block)
{           
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t offset = fs_start + (block * c->block_size);
    
//    printf("[FS] ERASE: %p, %d\n", offset, block);
        
    uint32_t ints = save_and_disable_interrupts();   
    multicore_lockout_start_blocking();     // pause another core
    flash_range_erase(offset, c->block_size);  
    multicore_lockout_end_blocking();
    restore_interrupts(ints);

    return 0;
}

int pico_sync(const struct lfs_config *c)
{
    return 0;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config PICO_FLASH_CFG = {
    // block device operations
    .read  = &pico_read,
    .prog  = &pico_prog,
    .erase = &pico_erase,
    .sync  = &pico_sync,

    // block device configuration
    .read_size = FLASH_PAGE_SIZE, // 256
    .prog_size = FLASH_PAGE_SIZE, // 256
    
    .block_size = BLOCK_SIZE_BYTES, // 4096
    .block_count = HW_FLASH_STORAGE_BYTES / BLOCK_SIZE_BYTES, // 352
    .block_cycles = 16, // ?
    
    .cache_size = FLASH_PAGE_SIZE, // 256
    .lookahead_size = FLASH_PAGE_SIZE,   // 256    
};

// Keyboard

static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

void process_kbd_report(hid_keyboard_report_t const *report) {

    int usbkey;

    if(menumode==0) { // Emulator mode

        enable_nmi=0;

        for(int i=0;i<7;i++) {  // Keymap 7 = Joypad
            keymap[i]=0xffff;
        }

        if(report->modifier&0x22) {  // SHIFT
            keymap[6]&=0xf7ff;
        }

        if(report->modifier&0x11) {  // CTRL
            keymap[6]&=0xfbff;
        }

        if(report->modifier&0x44) {  // ALT = Graph
            keymap[6]&=0xfdff;
        }

        for(int i=0;i<6;i++) {

            if ( report->keycode[i] ) {

                usbkey=report->keycode[i];
                if(sc3000usbcode[usbkey*2]) {
                    keymap[sc3000usbcode[usbkey*2+1]] &= ~sc3000usbcode[usbkey*2];
                }

            //  PAUSE = ESC

                if(usbkey==0x29) {
                    enable_nmi=1;
                }

            // Enter Menu
                if(usbkey==0x45) {
                    prev_report=*report;
                    menumode=1;
                    keypressed=0;
                }  
            }
        }

    prev_report=*report;

} else {  // menu mode

    for(uint8_t i=0; i<6; i++)
    {
        if ( report->keycode[i] )
        {
        if ( find_key_in_report(&prev_report, report->keycode[i]) )
        {
            // exist in previous report means the current key is holding
        }else
        {
            keypressed=report->keycode[i];
        }
        }
    } 
    prev_report = *report;
    }

}



// cart slots 

void cart_load(uint32_t cartno) {

    int32_t filesize;
    uint8_t cart_data;

        filesize=lfs_file_size(&lfs,&lfs_cart1);
        lfs_file_rewind(&lfs,&lfs_cart1);

        // printf("[Cart1 flash %d bytes]\n",filesize);
        // printf("[Cart1 erasing]\n");

        if(filesize>0x8000) filesize=0x8000;

        for(int i=0;i<filesize;i++) {

            lfs_file_read(&lfs,&lfs_cart1,&cart_data,1);
            mainram[i]=cart_data;

        }

}

//

static uint8_t mem_read(void *context,uint16_t address)
{

    return mainram[address];

}

static void mem_write(void *context,uint16_t address, uint8_t data)
{

    if(address>=0x8000) {
        mainram[address]=data;
    }

    return;

}

static uint8_t io_read(void *context, uint16_t address)
{
    uint8_t data = ioport[address&0xff];
    uint8_t b;
    uint32_t kanji_addr;

    address&=0xff;

// if((address&0xf0)!=0x90) {
// if(address!=0xa8) {
//     printf("[R:%x:%x]",Z80_PC(cpu),  address);

// }
// }

    switch(address) {

        case 0x7f: // DPSG

            return 0xff;

        case 0xbe:  // VDP Read

            return vrEmuTms9918ReadData(mainscreen);

        case 0xbf:  // VDP Status

            b=vrEmuTms9918ReadStatus(mainscreen);

            if(video_vsync==2) {
                b|=0x80;
                video_vsync=0;
            }

            return b;

        case 0xdc:  // i8s55 Port A (Keyboard)

//            printf("[Key%x:%x]",ioport[0xaa]&0xf,keymap[ioport[0xaa]&0xf]);

            return keymap[ioport[0xde]&0x7]&0xff;

        case 0xdd:  // i8s55 Port B (Keyboard)

//            printf("[Key%x:%x]",ioport[0xaa]&0xf,keymap[ioport[0xaa]&0xf]);

            b=(keymap[ioport[0xde]&0x7]>>8)&0xf;
            if(tapein()) {
                b|=0x80;
            }

            return b;

        default:
            break;

    }

    return ioport[address&0xff];

}

static void io_write(void *context, uint16_t address, uint8_t data)
{

//    printf("[%02x:%04x]",address&0xff,Z80_PC(cpu));
//    printf("[%02x:%02x]",address&0xff,data);


    uint8_t b;

    address&=0xff;

    switch(address) {

        case 0x7f:  // DPSG

            psg_write(data);
            return;

        case 0xbe:  // VDP Write
            vrEmuTms9918WriteData(mainscreen,data);
            return;

        case 0xbf:  // VDP control
            vrEmuTms9918WriteAddr(mainscreen,data);
            return;

        case 0xa0:  // PSG register
            ioport[0xa0]=data;
            return;

        case 0xde: // i8255 Port C

            if(data&0x8) {  // CMT Remote
                tape_ready=1;
                if(ioport[0xde]&0x10) {
                    tape_phase=0;
                }
            } else {
                tape_ready=0;
            }

            ioport[0xde]=data;
            return;

        case 0xdf:  // i8255 Control

            if((data&0x80)==0) { // Bit operation

                b=(data&0x0e)>>1;


                if(b==3) {  // CMT Remote
                    if(data&1) {
//printf("[CMT ON]");
                        tape_ready=1;
                        tape_phase=0;
                    } else {
//printf("[CMT OFF]");
                        tape_ready=0;

                    }
                }


                if(b==4) {  // Tape Out
                    tapeout(data&1);
                }


                if(data&1) {
                    ioport[0xde]|= 1<<b;
                } else {
                    ioport[0xde]&= ~(1<<b);
                }

            }


            return;

        default:
            break;
    }

    ioport[address&0xff]=data;

    return;

}

static uint8_t ird_read(void *context,uint16_t address) {

    // MSX Use Interrupt Mode 1

    z80_int(&cpu,FALSE);

    return 0xff;        // RST38

}

static void reti_callback(void *context) {

//    printf("RETI");

    // subcpu_enable_irq=0;
    // timer_enable_irq=0;
    // subcpu_irq_processing=0;
    // subcpu_command_processing=0;
    // subcpu_ird=0xff;

    // z80_int(&cpu,FALSE);

}

void init_emulator(void) {
//  setup emulator 

    // Initial Bank selection
    ioport[0xa8]=0;

    for(int i=0;i<8;i++) {
        keymap[i]=0xffff;
    }

    for(int i=0;i<8;i++) {
        megarom[i]=0;
    }

    psg_reset(0);

    tape_ready=0;
    tape_leader=0;

//    fdc_init();

    gamepad_info=0xffff;

}

void main_core1(void) {

    uint8_t bgcolor;
    uint32_t vramindex;

    multicore_lockout_victim_init();

    scanline=0;

    // set Hsync timer


    irq_set_exclusive_handler (PIO0_IRQ_0, hsync_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled (pio0, pis_interrupt0 , true);

    // set PSG timer
    // Use polling insted for I2S mode

    // add_repeating_timer_us(1000000/SAMPLING_FREQ,sound_handler,NULL  ,&timer2);

    while(1) { 

    }
}

int main() {

    uint32_t menuprint=0;
    uint32_t filelist=0;
    uint32_t subcpu_wait;

    static uint32_t hsync_wait,vsync_wait;

	// vreg_set_voltage(VREG_VOLTAGE_1_20);

    set_sys_clock_khz(DOTCLOCK * CLOCKMUL ,true);

    stdio_init_all();

    uart_init(uart0, 115200);

    initVGA();

    gpio_set_function(12, GPIO_FUNC_UART);
    gpio_set_function(13, GPIO_FUNC_UART);

    // gpio_set_slew_rate(0,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(1,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(2,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(3,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(4,GPIO_SLEW_RATE_FAST);

    gpio_set_drive_strength(2,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(3,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(4,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(5,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(6,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(7,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(8,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(9,GPIO_DRIVE_STRENGTH_2MA);

#ifdef USE_I2S
    i2s_init();
    i2s_dma_init();
#else
    // Beep & PSG

    gpio_set_function(10,GPIO_FUNC_PWM);
 //   gpio_set_function(11,GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(10);

    pwm_set_wrap(pwm_slice_num, 256);
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0);
//    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_B, 0);
    pwm_set_enabled(pwm_slice_num, true);

    // set PSG timer

    add_repeating_timer_us(1000000/SAMPLING_FREQ,sound_handler,NULL  ,&timer2);
#endif

    tuh_init(BOARD_TUH_RHPORT);


//    video_cls();

    video_hsync=0;
    video_vsync=0;

// uart handler

    // irq_set_exclusive_handler(UART0_IRQ,uart_handler);
    // irq_set_enabled(UART0_IRQ,true);
    // uart_set_irq_enables(uart0,true,false);

    multicore_launch_core1(main_core1);
    multicore_lockout_victim_init();

    sleep_ms(1);


// mount littlefs
    if(lfs_mount(&lfs,&PICO_FLASH_CFG)!=0) {
       cursor_x=0;
       cursor_y=0;
       fbcolor=7;
       video_print("Initializing LittleFS...");
       // format
       lfs_format(&lfs,&PICO_FLASH_CFG);
       lfs_mount(&lfs,&PICO_FLASH_CFG);
   }

    mainscreen=vrEmuTms9918New();
    menuscreen=vrEmuTms9918New();

    menuinit();

    menumode=1;  // Pause emulator

    init_emulator();

    cpu.read = mem_read;
    cpu.write = mem_write;
    cpu.in = io_read;
    cpu.out = io_write;
	cpu.fetch = mem_read;
    cpu.fetch_opcode = mem_read;
    cpu.reti = reti_callback;
    cpu.inta = ird_read;

    z80_power(&cpu,true);
    z80_instant_reset(&cpu);

    cpu_hsync=0;
    cpu_cycles=0;

#ifdef USE_FDC
    lfs_handler=lfs;
//    fdc_init(diskbuffer);

    fd_drive_status[0]=0;
#endif
    // start emulator
    
    menumode=1;

    while(1) {

        if(menumode==0) { // Emulator mode

        cpu_cycles += z80_run(&cpu,1);
        cpu_clocks++;

        // Wait

        if(enable_nmi) {
            enable_nmi=0;
            z80_nmi(&cpu);
        }

//        if((cpu_cycles-cpu_hsync)>1 ) { // 63us * 3.58MHz = 227

        if((!cpu_boost)&&(cpu_cycles-cpu_hsync)>227 ) { // 63us * 3.58MHz = 227

            while(video_hsync==0) ;
            cpu_hsync=cpu_cycles;
            video_hsync=0;
        }

        if((video_vsync==2)&&(cpu.iff1)) {
            if(vrEmuTms9918RegValue(mainscreen,TMS_REG_1)&0x20) { // VDP Enable interrupt
                z80_int(&cpu,true);
            }
        }

        if(video_vsync==1) { // Timer
            tuh_task();
            keymap[7]=gamepad_info;           
            video_vsync=2;
            vsync_scanline=scanline;         
            if((tape_autoclose)&&(save_enabled==2)) {
                if((cpu_cycles-tape_cycles)>TAPE_THRESHOLD) {
                    save_enabled=0;
                    lfs_file_close(&lfs,&lfs_file);
                }
            }

            if((tape_autoclose)&&(load_enabled==2)) {
                if((cpu_cycles-tape_cycles)>TAPE_THRESHOLD) {
                    load_enabled=0;
                    lfs_file_close(&lfs,&lfs_file);
                }
            }


        }

        } else { // Menu Mode


            unsigned char str[80];
            
            if(menuprint==0) {

                video_cls();
//                draw_menu();
                menuprint=1;
                filelist=0;
            }

            cursor_x=0;
            cursor_y=0;
            video_print("MENU");

            uint32_t used_blocks=lfs_fs_size(&lfs);
            sprintf(str,"Free:%d Blocks",(HW_FLASH_STORAGE_BYTES/BLOCK_SIZE_BYTES)-used_blocks);
            cursor_x=0;
            cursor_y=1;
            video_print(str);

            sprintf(str,"TAPE:%x",tape_ptr);
            cursor_x=0;
            cursor_y=2;
            video_print(str);


            cursor_x=3;            
            cursor_y=6;
            if(save_enabled==0) {
                video_print("SAVE: empty");
            } else {
                sprintf(str,"SAVE: %8s",tape_filename);
                video_print(str);
            }
            cursor_x=3;
            cursor_y=7;

            if(load_enabled==0) {
                video_print("LOAD: empty");
            } else {
                sprintf(str,"LOAD: %8s",tape_filename);
                video_print(str);
            }

            cursor_x=3;
            cursor_y=8;

            if(cart_loaded[0]==0) {
                video_print("Slot: empty");
            } else {
                sprintf(str,"Slot: %8s",cart1_filename);
                video_print(str);
            }

            cursor_x=3;
            cursor_y=9;
#ifdef USE_FDC
            if(fd_drive_status[0]==0) {
                video_print("FD: empty");
            } else {
                sprintf(str,"FD: %8s",fd_filename);
                video_print(str);
            }
#endif

            cursor_x=3;
            cursor_y=10;

            video_print("DELETE File");

            cursor_x=3;
            cursor_y=11;

            if(cpu_boost) {
                 video_print("CPU:Fast");
            } else {
                 video_print("CPU:Normal");
            }

            cursor_x=3;
            cursor_y=12;

            video_print("Reset");

            cursor_x=3;
            cursor_y=13;

            video_print("PowerCycle");

            cursor_x=0;
            cursor_y=menuitem+6;
            video_print("->");

   // for DEBUG ...

//            cursor_x=0;
//             cursor_y=23;
//                  sprintf(str,"%04x %04x %04x %04x %04x",Z80_PC(cpu),Z80_AF(cpu),Z80_BC(cpu),Z80_DE(cpu),Z80_HL(cpu));
// //                 sprintf(str,"%04x",Z80_PC(cpu));
//                  video_print(str);

            if(filelist==0) {
                draw_files(-1,0);
                filelist=1;
            }
     
            while(video_vsync==0);

            video_vsync=0;

                tuh_task();

                if(keypressed==0x52) { // Up
                    cursor_x=0;
                    cursor_y=menuitem+6;
                    video_print("  ");
                    keypressed=0;
                    if(menuitem>0) menuitem--;
#ifndef USE_FDC
                    if(menuitem==3) menuitem--;
#endif
                }

                if(keypressed==0x51) { // Down
                    cursor_x=0;
                    cursor_y=menuitem+6;
                    video_print("  ");
                    keypressed=0;
                    if(menuitem<7) menuitem++; 
#ifndef USE_FDC
                    if(menuitem==3) menuitem++;
#endif
                }

                if(keypressed==0x28) {  // Enter
                    keypressed=0;

                    if(menuitem==0) {  // SAVE
                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=enter_filename();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDWR|LFS_O_CREAT);
                                save_enabled=1;
                                tape_phase=0;
                                tape_ptr=0;
                                // tape_count=0;
                            }

                        } else if (save_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            save_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==1) { // LOAD
                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDONLY);
                                load_enabled=1;
                                tape_phase=0;
                                tape_ptr=0;
                                tape_suspend=0;
                                // tape_count=0;
//                                file_cycle=cpu.PC;
                            }
                        } else if(load_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            load_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==2) { // Slot Load

                        uint32_t res=file_selector();

                        if(res==0) {
                            memcpy(cart1_filename,filename,16);
                            lfs_file_open(&lfs,&lfs_cart1,cart1_filename,LFS_O_RDONLY);
                            cart_load(0);
                            cart_loaded[0]=1;
                            // if(cart_size_check(0)==0) {
                            //     if(cart_compare(0)!=0) {
                            //         cart_write(0);
                            //     }
                            //     cart_type_checker(0);
                            //     cart_loaded[0]=1;
                            // } else {
                            //     cart_loaded[0]=0;
                            // }
                            lfs_file_close(&lfs,&lfs_cart1);
                        }

                        menuprint=0;
                    }
#ifdef USE_FDC
                    if(menuitem==3) {  // FD
                        if(fd_drive_status[0]==0) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(fd_filename,filename,16);
                                lfs_file_open(&lfs,&fd_drive[0],fd_filename,LFS_O_RDONLY);
                                fdc_check(0);
                            }
                        } else {
                            lfs_file_close(&lfs,&fd_drive[0]);
                            fd_drive_status[0]=0;
                        }
                        menuprint=0;
                    }
#endif
                    if(menuitem==4) { // Delete

                        if((load_enabled==0)&&(save_enabled==0)) {
                            uint32_t res=enter_filename();

                            if(res==0) {
                                lfs_remove(&lfs,filename);
                            }
                        }

                        menuprint=0;

                    }

                    if(menuitem==5) { 
                        cpu_boost++;
                        if(cpu_boost>1) cpu_boost=0;
                        menuprint=0;
                    }

                    if(menuitem==6) { // Reset
                        menumode=0;
                        menuprint=0;
                    
                        init_emulator();
                        z80_power(&cpu,true);

                    }

                    if(menuitem==7) { // PowerCycle
                        menumode=0;
                        menuprint=0;

                        memset(mainram+0x8000,0,0x8000);
                        memset(ioport,0,0x100);

                        init_emulator();

//                        z80_instant_reset(&cpu);
                        z80_power(&cpu,true);

                    }

                }

                if(keypressed==0x45) {
                    keypressed=0;
                    menumode=0;
                    menuprint=0;
                //  break;     // escape from menu
                }

        }


    }

}
