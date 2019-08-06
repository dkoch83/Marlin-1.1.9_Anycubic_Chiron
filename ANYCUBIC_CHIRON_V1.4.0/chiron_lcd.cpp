/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * chiron_lcd.cpp
 *
 * LCD implementation for Anycubic Chiron LCD
 */
#include "MarlinConfig.h"

#if ENABLED(CHIRON_LCD)

#include "chiron_lcd.h"
#include "music.h"

#if ENABLED(SDSUPPORT)
  #include "cardreader.h"
  #include "SdFatConfig.h"
#else
  #define LONG_FILENAME_LENGTH 0
#endif

#include "temperature.h"
#include "planner.h"
#include "stepper.h"
#include "duration_t.h"
#include "printcounter.h"
#include "parser.h"
#include "configuration_store.h"

#include "Marlin.h"

#if ENABLED(POWER_LOSS_RECOVERY) || ENABLED(CHIRON_POWER_LOSS_RECOVERY)
	#include "power_loss_recovery.h"
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
	#include "runout.h"
#endif

#if USE_MARLINSERIAL
  // Make an exception to use HardwareSerial too
  #undef HardwareSerial_h
  #include <HardwareSerial.h>
  #define USB_STATUS true
#else
  #define USB_STATUS Serial
#endif

#define LCD_SERIAL Serial3

// This is based on longest sys command + a filename, plus some buffer
// in case we encounter some data we don't recognize
// There is no evidence a line will ever be this long, but better safe than sorry
#define MAX_TFT_BUFFER 32

#if PIN_EXISTS(SD_DETECT)
	uint8_t lcd_sd_status;
#endif

#if HAS_RESUME_CONTINUE
	extern volatile bool wait_for_user;
#endif

extern uint8_t commands_in_queue;

//void lcd_reset_status();

// #########################################
//            Anycubi Chiron
// #########################################
void get_command_from_TFT();
void SDCARD_UPDATA();

void restore_z_values();

void pauseCMDsend();
void setupSDCARD();
void TFT_Commond_Scan();
void setupMyZoffset();
void Endstopsbeep();
void mybeep(int beepP,int beepS);
void get_command_from_TFT(const char* command);

bool TFTcode_seen(char code);
float TFTcode_value();

void Newok_to_send();
void setuplevelTest();
void USBOnLineTest();
void FilamentScan();
void SetupFilament();
void setup_ChironPowerLossPin();
void SaveWay2Leveling();
void ReadWay2Leveling();	

#if PIN_EXISTS(SD_DETECT)
	extern uint8_t lcd_sd_status;
#endif

#if HAS_BED_PROBE
	float SAVE_zprobe_zoffset;
#endif

bool FilamentRunOut = false;
bool pauseCMDsendflag = false;
char PointTestFlag = 0;
uint16_t filenumber;

bool ReadMyfileNrFlag = true;
unsigned char Manual_Leveling;

static boolean TFTcomment_mode = false;
static char *TFTstrchr_pointer;

const unsigned int Max_ModelCooling = MAX_MODEL_COOLING_PRECENT_VALUE * 255;      
char sdcardstartprintingflag = 0;

// File List
uint16_t MyFileNrCnt	= 0;
uint16_t fileoutputcnt	= 0;
uint16_t filecnt		= 0;

// USB Connection
bool USBConnectFlag 	= false;
bool UsbOnLineFlag  	= false;
 
// Track incoming command bytes from the LCD
int inbound_count = 0;

// For sending print completion messages
bool last_printing_status = false;

// Everything written needs the high bit set.
void write_to_lcd_P(const char * const message) {
	char encoded_message[MAX_CMD_SIZE];
	uint8_t message_length = MIN(strlen_P(message), sizeof(encoded_message));

	for (uint8_t i = 0; i < message_length; i++)
		encoded_message[i] = pgm_read_byte(&message[i]);

	LCD_SERIAL.Print::write(encoded_message, message_length);
}

void write_to_lcd(const char * const message) {
  LCD_SERIAL.Print::write(message, strlen(message));
}

void write_to_lcd_f(float value) {
  LCD_SERIAL.Print::print(value, 2);
}

void write_to_lcd_i(int value) {
  LCD_SERIAL.Print::print(value, DEC);
}

void lcd_init() {
	inbound_count = 0;
	LCD_SERIAL.begin(115200);

	#ifdef AUTO_BED_LEVELING_BILINEAR
		setupMyZoffset();
		delay(10);
	#endif

	_delay_ms(20);
	PowerOnMusic();
	setup_ChironPowerLossPin();
	setupSDCARD();
	SetupFilament();
	
	sdcardstartprintingflag 	= 0;
	last_printing_status 		= false;
	pauseCMDsendflag 			= false;
	MyFileNrCnt 				= 0;
	
	// USB Connection
	USBConnectFlag 				= 0;
	UsbOnLineFlag  				= false;
	
	_delay_ms(10);  // wait 1sec to display the splash screen
	
	// Signal init
	write_to_lcd_P(PSTR("\r\n"));
	write_to_lcd_P(PSTR("J17\r\n")); // J17 main board reset
	delay(10);
	write_to_lcd_P(PSTR("J12\r\n")); //  READY	
}

void lcd_update() {
	static char inbound_buffer[MAX_TFT_BUFFER];  
	static unsigned int counter=0;
	static unsigned int Scancount=0;
	
	USBOnLineTest();
	pauseCMDsend();
	FilamentScan();
  
	counter++;
	if(counter%1000==0) {
		counter=0;  
	SDCARD_UPDATA();
	} 
	
	// Test Print Finish
	if (sdcardstartprintingflag == 1 && card.isFileOpen() == false ) {
		sdcardstartprintingflag = 0;
		last_printing_status = false;
		
		write_to_lcd_P(PSTR("J14\r\n")); //PRINT DONE
	}
	
	
	// T0 Test
	if( (thermalManager.degHotend(0) < 5) || ((thermalManager.degHotend(0) > 280)) ) Scancount++;
	if( Scancount > 61000 ) { 
		Scancount = 0;
		write_to_lcd_P(PSTR("J10\r\n")); // T0 unnormal  
	} 

	while (LCD_SERIAL.available()) {
		const byte b = (byte)LCD_SERIAL.read();
		if( b == '\n' || b == '\r' || (b == ':' && TFTcomment_mode == false) || inbound_count >= (sizeof(inbound_buffer) - 1) ) {	
			
			if(!inbound_count) { 				//if empty line
				TFTcomment_mode = false; 		//for new command
				return;
			}
			inbound_buffer[inbound_count] = 0; 	//terminate string
			
			// Debug
			//SERIAL_ECHOLN(inbound_buffer);
			
			if(!TFTcomment_mode){
				get_command_from_TFT(inbound_buffer);
				LCD_SERIAL.flush();
			}
			inbound_count = 0; 					// clear buffer
		} else {
			if(b == ';') TFTcomment_mode = true;
			if(!TFTcomment_mode) {
				inbound_buffer[inbound_count++] = b;
			}
		}   
	}
}

/*
*################################
*		void pauseCMDsend()
*################################
*/

void pauseCMDsend() {
	static char temp = 0;
	char cmd[32];
	if( commands_in_queue < BUFSIZE && pauseCMDsendflag == true) { 
		temp++;
		if(temp == 1) {
			#if ENABLED(PARK_HEAD_ON_PAUSE)
			enqueue_and_echo_commands_P(PSTR("M125"));
			// when pause sd printing,send "ok" to tft as read buffer carry out
			planner.synchronize();
			write_to_lcd_P(PSTR("J18\r\n")); // pausing done
			pauseCMDsendflag = false;
			temp = 0;
			#else
			enqueue_and_echo_commands_P(PSTR("G91"));
			#endif
		}
		if(temp == 2) {
			if (axis_relative_modes[E_CART] == false) {
				sprintf_P(cmd, PSTR("G1 E%fF2400.00000"), (current_position[E_CART]-5.0));
				enqueue_and_echo_command_now(cmd);
				} else {
				enqueue_and_echo_commands_P(PSTR("G1 E-5"));
			}
		}
		if(temp == 3) {
			enqueue_and_echo_commands_P(PSTR("G1 Z+10"));
			pauseCMDsendflag = false;
			temp = 0;
			// when pause sd printing,send "ok" to tft as read buffer carry out
			planner.synchronize();
			write_to_lcd_P(PSTR("J18\r\n")); // pausing done
		}
	}
}

void FilamentScan() {
	static uint8_t count = 0;
	
	// Reset Counter
	if(card.sdprinting == true && wait_for_user == false) {
		count = 0;
	}

	if(last_printing_status == false && READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_INVERTING) {
		write_to_lcd_P(PSTR("J15\r\n")); 	// J15 FILAMENT LACK
	}	
	
	if(wait_for_user == true && card.sdprinting == false && READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_INVERTING && count == 0) {
		FilamentLack(); //music
		FilamentRunOut = true;
		write_to_lcd_P(PSTR("J23\r\n")); 	// J23 FILAMENT LACK with the prompt box don't disappear
		write_to_lcd_P(PSTR("J18\r\n"));    // pausing done
		count++;
	} else 	
	if(wait_for_user == true 
		&& card.sdprinting == false 
		&& READ(FIL_RUNOUT_PIN) == !FIL_RUNOUT_INVERTING
		&& thermalManager.is_heater_idle(0) == false
		&& FilamentRunOut == false
		&& count == 1
	) {
		write_to_lcd_P(PSTR("J06\r\n"));  	// Message hotend heating
		wait_for_user = false;
		count++;
	} else 
	if (wait_for_heatup == false && count == 3) {
		write_to_lcd_P(PSTR("J07\r\n")); 	//hotend heating done
	}			
} 

/*
void lcd_reset_status() {
    if (wait_for_heatup) {
		write_to_lcd_P(PSTR("J09\r\n")); //hotbed heating
    }
}
*/

void lcd_setalertstatusPGM(const char* message) {
	/*
	char message_buffer[MAX_CURLY_COMMAND];
	sprintf_P(message_buffer, PSTR("{E:%s}"), message);
	write_to_lcd(message_buffer);
	*/
}

void SDCARD_UPDATA() {
	uint8_t sd_status = IS_SD_INSERTED();
	if (sd_status != lcd_sd_status) {
		if (sd_status) {
			card.initsd();
			MyFileNrCnt=0;
			write_to_lcd_P(PSTR("J00\r\n"));
		} else {
			card.release();
			write_to_lcd_P(PSTR("J01\r\n"));
		}
		lcd_sd_status = sd_status;
	}  
}

void setup_ChironPowerLossPin() {
    pinMode(POWER_LOSS_PIN, INPUT);
    pinMode(POWER_LOSS_CON_PIN, OUTPUT);
	WRITE(POWER_LOSS_CON_PIN, HIGH);  // ON
}

void SetupFilament() {
    pinMode(FIL_RUNOUT_PIN,INPUT);
    WRITE(FIL_RUNOUT_PIN,HIGH);
     _delay_ms(50);
}

void USBOnLineTest() {
    static long int temp = 0;
    if(USBConnectFlag == false) {
		if(UsbOnLineFlag == true) {
		temp++;
		UsbOnLineFlag = false;
			if(temp > 1) {              
			  USBConnectFlag = true;
			  write_to_lcd_P(PSTR("J03\r\n")); 	//usb connect
			  temp = 0;
			}    
		}
    } else if(USBConnectFlag == true) {
		if(UsbOnLineFlag == false) {
			temp++;
			if(temp > 50000) {          
			  UsbOnLineFlag  = false;
			  USBConnectFlag = false;
			  write_to_lcd_P(PSTR("J12\r\n")); //ready
			  temp=0;
			}
		}
		else { 
			temp = 0;
			UsbOnLineFlag = false;
		}
    }      
}

#define Z_TEST 2
void setuplevelTest() {
    pinMode(Z_TEST,INPUT);
    WRITE(Z_TEST, HIGH);
    pinMode(BEEPER_PIN,OUTPUT);
    WRITE(BEEPER_PIN, LOW);
}

void Newok_to_send() {
  previous_move_ms = millis();
}

/*
void NEWFlushSerialRequestResend() {
  NewSerial.flush();
  Newok_to_send();
}
*/

void TFT_Commond_Scan() {
	lcd_update();
}

float TFTcode_value() {
	return (strtod(TFTstrchr_pointer + 1, NULL));
}

bool TFTcode_seen(const char* command, char code) {
  TFTstrchr_pointer = strchr(command, code);
  return (TFTstrchr_pointer != NULL);  //Return True if a character was found
}

void get_command_from_TFT(const char* TFTcmdbuffer) {
  char cmd[30];
  char *starpos = NULL;
  unsigned int temp;
  uint8_t x;
  uint8_t y;
  
	TFTcomment_mode = false; //for new command

	if((strchr(TFTcmdbuffer, 'A') != NULL)){
	TFTstrchr_pointer = strchr(TFTcmdbuffer, 'A');
		switch((int)((strtod(&TFTcmdbuffer[TFTstrchr_pointer - TFTcmdbuffer + 1], NULL)))) {
        case 0: // A0 GET HOTEND TEMP 
			write_to_lcd_P(PSTR("A0V "));
			write_to_lcd(itostr3(int(thermalManager.degHotend(0) + 0.5)));
			write_to_lcd_P(PSTR("\r\n"));
        break;
                          
        case 1: // A1  GET HOTEND TARGET TEMP
			write_to_lcd_P(PSTR("A1V "));
			write_to_lcd(itostr3(int(thermalManager.degTargetHotend(0) + 0.5)));
			write_to_lcd_P(PSTR("\r\n"));
        break; 
               
        case 2: // A2 GET HOTBED TEMP
			write_to_lcd_P(PSTR("A2V "));
			write_to_lcd(itostr3(int(thermalManager.degBed() + 0.5)));
			write_to_lcd_P(PSTR("\r\n"));
        break;                     

        case 3: // A3 GET HOTBED TARGET TEMP
			write_to_lcd_P(PSTR("A3V "));
			write_to_lcd(itostr3(int(thermalManager.degTargetBed() + 0.5)));
			write_to_lcd_P(PSTR("\r\n"));
        break;
            
        case 4: // A4 GET FAN SPEED 
			temp = ((fanSpeeds[0]*100)/Max_ModelCooling+1);
			temp = constrain(temp,0,100);
			write_to_lcd_P(PSTR("A4V "));
			write_to_lcd_i(temp);
			write_to_lcd_P(PSTR("\r\n"));
        break;
        
        case 5: // A5 GET CURRENT COORDINATE 
			write_to_lcd_P(PSTR("A5V "));
			write_to_lcd_P(PSTR("X: "));
			write_to_lcd_f(current_position[X_AXIS]);
			write_to_lcd_P(PSTR(" "));		
			write_to_lcd_P(PSTR("Y: "));
			write_to_lcd_f(current_position[Y_AXIS]);
			write_to_lcd_P(PSTR(" "));		
			write_to_lcd_P(PSTR("Z: "));
			write_to_lcd_f(current_position[Z_AXIS]);
			write_to_lcd_P(PSTR(" "));	
			write_to_lcd_P(PSTR("\r\n"));
        break;
        
        case 6: // A6 GET SD CARD PRINTING STATUS
			if(card.sdprinting) {
				write_to_lcd_P(PSTR("A6V "));
				if(card.cardOK) {
					write_to_lcd(itostr3(card.percentDone()));
				} else {
					write_to_lcd_P(PSTR("J02"));
				}				
			} else {
				write_to_lcd_P(PSTR("A6V ---"));
			}
			write_to_lcd_P(PSTR("\r\n")); 	// Enter
        break;                          

        case 7: // A7 GET PRINTING TIME
			write_to_lcd_P(PSTR("A7V "));
			if(print_job_timer.duration() != 0) { 	// print time
				write_to_lcd(itostr3(print_job_timer.duration()/3600));
				write_to_lcd_P(PSTR(" H "));	
				write_to_lcd(itostr3((print_job_timer.duration()/60)%60));
				write_to_lcd_P(PSTR(" M"));
			} else {
				write_to_lcd_P(PSTR(" 999:999"));
			}			
			write_to_lcd_P(PSTR("\r\n")); // Enter
        break;                      
        
        case 8: // A8 GET SD LIST
			if(!IS_SD_INSERTED()) {
				MyFileNrCnt=0;
				write_to_lcd_P(PSTR("J02\r\n"));
			} else { 
				if ( MyFileNrCnt == 0 ) {
					card.chdir(PSTR(""));
					MyFileNrCnt = card.get_num_Files();
				} 
				if ( MyFileNrCnt > 0 ) {
					if(TFTcode_seen(TFTcmdbuffer, 'S')) filenumber = TFTcode_value();
					write_to_lcd_P(PSTR("FN \r\n"));
					
					fileoutputcnt = 0;
					filecnt = 0;
					while ( (fileoutputcnt < (filenumber + 4)) && (filecnt <  (MyFileNrCnt)) ) {
						card.getfilename((filecnt),NULL);
						if( (strstr(card.filename,".g") !=NULL ) || (strstr(card.filename,".G") !=NULL) ) {
							fileoutputcnt++;
							if (fileoutputcnt > filenumber) {
								//write_to_lcd(prepend);
								write_to_lcd(card.filename);
								write_to_lcd_P(PSTR("\r\n"));
								if (card.longFilename[0] == '\0' ) write_to_lcd(card.filename);
								else write_to_lcd(card.longFilename);
								write_to_lcd_P(PSTR("\r\n"));			
							}
						}
						filecnt++;
					}
					write_to_lcd_P(PSTR("END\r\n"));					
				}
			}
        break;
        
        case 9: // A9 pause sd
			if(card.sdprinting) {
				write_to_lcd_P(PSTR("J05\r\n")); // J05 Pausing
				card.pauseSDPrint();
				print_job_timer.pause();
				pauseCMDsendflag = true;
			} else {
				write_to_lcd_P(PSTR("J16\r\n")); // J16, if status error, send stop print flag in case TFT no response
			}
        break;

        case 10: // A10 resume sd print
        if(card.sdprinting == false && wait_for_user == false && FilamentRunOut == false) {
			planner.synchronize();
			#if ENABLED(PARK_HEAD_ON_PAUSE)
				enqueue_and_echo_commands_P(PSTR("M24"));
			#else
				enqueue_and_echo_commands_P(PSTR("G91"));
				if (axis_relative_modes[E_CART] == false) {
					sprintf_P(cmd, PSTR("G1 E%fF2400.00000"), (current_position[E_CART]+5.0));
					enqueue_and_echo_command_now(cmd);
					} else {
					enqueue_and_echo_commands_P(PSTR("G1 E5\n"));
				}
				enqueue_and_echo_commands_P(PSTR("G1 Z-10"));
				enqueue_and_echo_commands_P(PSTR("G90"));

				last_printing_status = true;
				print_job_timer.start();
				card.startFileprint();
			#endif
			pauseCMDsendflag = false;
		} else if (card.sdprinting == false && wait_for_user == true && READ(FIL_RUNOUT_PIN) == !FIL_RUNOUT_INVERTING) {
			FilamentRunOut = false;
			wait_for_user = false;
		} else {
			FilamentLack(); //music
			FilamentRunOut = true;
			write_to_lcd_P(PSTR("J23\r\n")); 	// J23 FILAMENT LACK with the prompt box don't disappear
			write_to_lcd_P(PSTR("J18\r\n"));    // pausing done
			break;
		}
        write_to_lcd_P(PSTR("J04\r\n")); // J04 ok printing form sd card
        break;
       
        case 11: // A11 STOP SD PRINT
			if( (card.isFileOpen() == true ) ) {    
				card.abort_sd_printing = true;
				last_printing_status = false;
				write_to_lcd_P(PSTR("J16\r\n")); // Stop
				print_job_timer.stop();
			}
        break;
                               
        case 12: // A12 kill
          //    NEW_SERIAL_PROTOCOLPGM("J11");//kill()
          //    TFT_SERIAL_ENTER();
          //    kill();
        break;
        
        case 13: // A13 SELECTION FILE
			if( (!planner.movesplanned())&&(!last_printing_status) ) {
				starpos = (strchr(TFTstrchr_pointer + 4,'*'));
				if( starpos!=NULL) *(starpos-1) = '\0';
				//write_to_lcd_P(PSTR("/"));
				card.openFile(TFTstrchr_pointer + 4, true, false);
				if ( card.isFileOpen() ) {
					write_to_lcd_P(PSTR("J20\r\n")); 	// OPEN SUCCESS
					sdcardstartprintingflag = 1;

					write_to_lcd(TFTstrchr_pointer + 4);
					write_to_lcd_P(PSTR("\r\n"));
				} else {
					write_to_lcd_P(PSTR("J21\r\n")); 	// OPEN FAIL

					sdcardstartprintingflag = 0;
				}
				write_to_lcd_P(PSTR("\r\n")); // Enter
			}
        break;

        case 14: // A14 START PRINTING
			if( !planner.movesplanned() && !last_printing_status ) {
				errorFlag 		= 0;
				UsbOnLineFlag	= false;
				print_job_timer.reset();		
				card.startFileprint();
				last_printing_status = true;
				print_job_timer.start();
				write_to_lcd_P(PSTR("J06\r\n")); // hotend heating 
			}
        break;
        
        case 15: // A15 Power loss recovery resume
		{
			#if defined(CHIRON_POWER_LOSS_RECOVERY)
			if( (!planner.movesplanned()) && (!last_printing_status) ) {
				enqueue_and_echo_commands_P(PSTR("G28 R2"));			  
			  
				#if HAS_HEATED_BED
				const int16_t bt = job_recovery_info.target_temperature_bed;
				if (bt) {
				  // Restore the bed temperature
				  sprintf_P(cmd, PSTR("M190 S%i"), bt);
				  enqueue_and_echo_command(cmd);
				}
				#endif
				
				// Restore all hotend temperatures
				HOTEND_LOOP() {
					const int16_t et = job_recovery_info.target_temperature[e];
					if (et) {
					  sprintf_P(cmd, PSTR("M109 S%i"), et);
					  enqueue_and_echo_command(cmd);
					}
				}

				// Restore print cooling fan speeds
				for (uint8_t i = 0; i < FAN_COUNT; i++) {
					int16_t f = job_recovery_info.fanSpeeds[i];
					if (f) {
						sprintf_P(cmd, PSTR("M106 P%i S%i"), i, f);
						enqueue_and_echo_command(cmd);
					}
				}
				last_printing_status = true;
				// Start draining the job recovery command queue
				job_recovery_phase = JOB_RECOVERY_YES;
				write_to_lcd_P(PSTR("OK")); // hotend heating
			}
			#endif
			write_to_lcd_P(PSTR("\r\n")); // Enter
		}
        break;

        case 16: // A16 set hotend temp
			if(TFTcode_seen(TFTcmdbuffer, 'S')) {
				temp = constrain(TFTcode_value(), 0, 275);       
				thermalManager.setTargetHotend(temp, 0);  
			} else if((TFTcode_seen(TFTcmdbuffer, 'C')) && (!planner.movesplanned())) {
				temp = constrain(TFTcode_value(), 0, 275);       
				thermalManager.setTargetHotend(temp, 0);  
			}
        break;
        
        case 17: // A17 set hotbed temp
			if(TFTcode_seen(TFTcmdbuffer, 'S')) {
				temp = constrain(TFTcode_value(), 0, 120);
				thermalManager.setTargetBed(temp);
			}
        break;
        
        case 18: // A18 set fan speed
			if (TFTcode_seen(TFTcmdbuffer, 'S')) {
				temp = (TFTcode_value() * Max_ModelCooling / 100);
				temp = constrain(temp, 0, Max_ModelCooling);
				fanSpeeds[0] = temp;
			} else fanSpeeds[0] = Max_ModelCooling;
			write_to_lcd_P(PSTR("\r\n")); // Enter                         
        break;
        
        case 19: // A19 CLOSED STEPER DIRV
			if((!USBConnectFlag)&&(!card.sdprinting)) {
				quickstop_stepper(); 
				disable_X();
				disable_Y();
				disable_Z();
				disable_E0();
			}
			write_to_lcd_P(PSTR("\r\n")); // Enter
        break;
        
        case 20: // A20 read printing speed
			if (TFTcode_seen(TFTcmdbuffer, 'S')) {
				feedrate_percentage = constrain(TFTcode_value(), 40, 999);
			} else {
				write_to_lcd_P(PSTR("A20V "));
				write_to_lcd_i(feedrate_percentage);                            
				write_to_lcd_P(PSTR("\r\n")); // Enter
			}
        break;
        
        case 21: // A21 Home all
			if((!planner.movesplanned()) && (!last_printing_status) && (!card.sdprinting)) {
				if( TFTcode_seen(TFTcmdbuffer, 'X') || TFTcode_seen(TFTcmdbuffer, 'Y') || TFTcode_seen(TFTcmdbuffer, 'Z') ) {
					if( TFTcode_seen(TFTcmdbuffer, 'X') ) enqueue_and_echo_commands_P(PSTR("G28 X"));
					if( TFTcode_seen(TFTcmdbuffer, 'Y') ) enqueue_and_echo_commands_P(PSTR("G28 Y"));
					if( TFTcode_seen(TFTcmdbuffer, 'Z') ) enqueue_and_echo_commands_P(PSTR("G28 Z"));
				} else if (TFTcode_seen(TFTcmdbuffer, 'C')) enqueue_and_echo_commands_P(PSTR("G28"));                     
			}                          
        break;

        case 22: // A22 move X/Y/Z
			if((!planner.movesplanned()) && (!last_printing_status) && (!card.sdprinting)) {
				float coorvalue;
				unsigned int movespeed=0;
				if(TFTcode_seen(TFTcmdbuffer, 'F')) movespeed = TFTcode_value(); 
				enqueue_and_echo_commands_P(PSTR("G91"));     

				if(TFTcode_seen(TFTcmdbuffer, 'X')) {
					coorvalue=TFTcode_value(); 
					if ((coorvalue <= 0.2) && coorvalue > 0) {
						sprintf_P(cmd, PSTR("G1 X0.1F%i"), movespeed);
						enqueue_and_echo_command_now(cmd);
					} else if ((coorvalue <= -0.1) && coorvalue > -1) {
						sprintf_P(cmd, PSTR("G1 X-0.1F%i"), movespeed);
						enqueue_and_echo_command_now(cmd);
					} else {
						sprintf_P(cmd, PSTR("G1 X%iF%i"), int(coorvalue), movespeed);
						enqueue_and_echo_command_now(cmd);
					}                      
				} else if(TFTcode_seen(TFTcmdbuffer, 'Y')) {
					coorvalue=TFTcode_value();
					if ((coorvalue <= 0.2) && coorvalue > 0) {
						sprintf_P(cmd, PSTR("G1 Y0.1F%i"), movespeed);
						enqueue_and_echo_command_now(cmd);
					} else if ((coorvalue <= -0.1) && coorvalue > -1) {
						sprintf_P(cmd, PSTR("G1 Y-0.1F%i"), movespeed);
						enqueue_and_echo_command_now(cmd);
					} else {
						sprintf_P(cmd, PSTR("G1 Y%iF%i"), int(coorvalue), movespeed); 
						enqueue_and_echo_command_now(cmd); 
					}                                  
				} else if(TFTcode_seen(TFTcmdbuffer, 'Z')) {
					coorvalue=TFTcode_value();
					if ((coorvalue<=0.2)&&coorvalue>0) {
						sprintf_P(cmd, PSTR("G1 Z0.1F%i"), movespeed);
						enqueue_and_echo_command_now(cmd);
					} else if ((coorvalue<=-0.1) && coorvalue > -1) {
						sprintf_P(cmd, PSTR("G1 Z-0.1F%i"), movespeed);
						enqueue_and_echo_command_now(cmd);
					} else {
					sprintf_P(cmd, PSTR("G1 Z%iF%i"), int(coorvalue), movespeed);
					enqueue_and_echo_command_now(cmd); 
					}                                     
				} else if(TFTcode_seen(TFTcmdbuffer, 'E')) {
					coorvalue=TFTcode_value();
					if ((coorvalue<=0.2) && coorvalue>0) {
						sprintf_P(cmd,PSTR("G1 E0.1F%i"), movespeed);
						enqueue_and_echo_command_now(cmd);
					} else if ((coorvalue <= -0.1) && coorvalue > -1) {
						sprintf_P(cmd,PSTR("G1 E-0.1F%i"),movespeed);
						enqueue_and_echo_command_now(cmd);
					} else {
						sprintf_P(cmd, PSTR("G1 E%iF500"), int(coorvalue));
						enqueue_and_echo_command_now(cmd);
					}  
				}
				enqueue_and_echo_commands_P(PSTR("G90"));
			}
			write_to_lcd_P(PSTR("\r\n")); // Enter                          
        break;
                                  
        case 23: // A23 prheat pla
			if((!planner.movesplanned()) && (!last_printing_status)) {
				thermalManager.setTargetBed(PREHEAT_1_TEMP_BED);
				thermalManager.setTargetHotend(PREHEAT_1_TEMP_HOTEND, 0);				
				write_to_lcd_P(PSTR("OK\r\n"));
			}
        break;

        case 24: // A24 prheat abs
			if((!planner.movesplanned())&&(!last_printing_status)) {
				thermalManager.setTargetBed(PREHEAT_2_TEMP_BED);
				thermalManager.setTargetHotend(PREHEAT_2_TEMP_HOTEND, 0);
				write_to_lcd_P(PSTR("OK\r\n"));
			}
        break;
        
		case 25: //A25 cool down
			if((!planner.movesplanned())&&(!last_printing_status)) {
				thermalManager.setTargetHotend(0,0);
				thermalManager.setTargetBed(0);
				write_to_lcd_P(PSTR("J12\r\n"));
			}
		break;

		case 26: // A26 refresh
			card.initsd();
			MyFileNrCnt = 0;
			if(!IS_SD_INSERTED()){
				write_to_lcd_P(PSTR("J02\r\n"));
			}
		break;
       
		#ifdef SERVO_ENDSTOPS
		/*
		case 27: // A27 servos angles  adjust  
		  if((!planner.movesplanned())&&(!last_printing_status)) {
			char value[30];
			planner.buffer_line(current_position[X_AXIS],current_position[Y_AXIS], 20, current_position[E_AXIS], 10, active_extruder);
			stepper.synchronize();
			NEW_SERIAL_PROTOCOLPGM("A27V ");
			NEW_SERIAL_PROTOCOLPGM("R ");
			NEW_SERIAL_PROTOCOL(RiseAngles);
			TFT_SERIAL_SPACE();
			NEW_SERIAL_PROTOCOLPGM("F ");
			NEW_SERIAL_PROTOCOL(FallAngles);
			TFT_SERIAL_SPACE();                          
		   if(TFTcode_seen(TFTcmdbuffer, 'R')) {
			   RiseAngles = TFTcode_value();
		   }
		   if(TFTcode_seen(TFTcmdbuffer, 'F')) {
			   FallAngles = TFTcode_value();
		   }   
		   if(TFTcode_seen(TFTcmdbuffer, 'O')){ 
			  SaveMyServoAngles();
			  delay(200);
			  servos[0].detach();
		   }                 
		 }
		TFT_SERIAL_ENTER();
		break;
		*/
		#endif

		case 28: // A28 filament test
			if(TFTcode_seen(TFTcmdbuffer, 'O')) {
			} else if(TFTcode_seen(TFTcmdbuffer, 'C'));
			write_to_lcd_P(PSTR("\r\n")); // Enter
		break;   
       
		case 29: // A29 bed grid read
		{
			#ifdef AUTO_BED_LEVELING_BILINEAR
			if(TFTcode_seen(TFTcmdbuffer, 'X')) x = TFTcode_value();
			if(TFTcode_seen(TFTcmdbuffer, 'Y')) y = TFTcode_value();
			float Zvalue = z_values[x][y];
			Zvalue = Zvalue * 100;

			write_to_lcd_P(PSTR("A29V "));
			write_to_lcd_f(Zvalue);
			write_to_lcd_P(PSTR("\r\n")); // Enter
			#endif
		}
		break;
		
		case 30: // A30 auto leveling
			#ifdef AUTO_BED_LEVELING_BILINEAR
			if((planner.movesplanned())||(card.sdprinting)) {
				write_to_lcd_P(PSTR("J24\r\n"));	// forbid auto leveling
				} else {
				write_to_lcd_P(PSTR("J26\r\n"));	// start auto leveling
			}
			if(TFTcode_seen(TFTcmdbuffer, 'S') ) {
				enqueue_and_echo_commands_P(PSTR("G28\nG29"));
			}
			#else
			write_to_lcd_P(PSTR("J24\r\n"));	// forbid auto leveling
			#endif
		break;
		
		case 31: // A31 zoffset set get or save
			//if(Manual_Leveling==0xaa)break;
			#ifdef AUTO_BED_LEVELING_BILINEAR
			if(TFTcode_seen(TFTcmdbuffer, 'S')) {
				float value = constrain(TFTcode_value(),-1.0,1.0);
				zprobe_zoffset += value;
				for (x = 0; x < GRID_MAX_POINTS_X; x++) {
					for (y = 0; y < GRID_MAX_POINTS_Y; y++) z_values[x][y] += value;
				}
				write_to_lcd_P(PSTR("A31V "));
				write_to_lcd_f(zprobe_zoffset);
				refresh_bed_level();
			}
			if(TFTcode_seen(TFTcmdbuffer, 'G')) {
				SAVE_zprobe_zoffset = zprobe_zoffset;
				write_to_lcd_P(PSTR("A31V "));
				write_to_lcd_f(SAVE_zprobe_zoffset);
			}
			if(TFTcode_seen(TFTcmdbuffer, 'D')) {
				SAVE_zprobe_zoffset = zprobe_zoffset;
				settings.save();
				set_bed_leveling_enabled(true);
				refresh_bed_level();
			}
			write_to_lcd_P(PSTR("\r\n")); // Enter
			#endif
		break;
       
		case 32: // A32 clean leveling beep flag
		break;
       
		case 33: // A33 get version info
			if(errorFlag==0) {
				write_to_lcd_P(PSTR("J33 "));
				write_to_lcd_P(PSTR(MSG_MY_VERSION));
				write_to_lcd_P(PSTR("\r\n")); // Enter					
			} else if(errorFlag==1) {
				write_to_lcd_P(PSTR("J33 "));
				write_to_lcd_P(PSTR("ReadSD card error!"));
				write_to_lcd_P(PSTR("\r\n")); // Enter					
			} else if(errorFlag==2) {
				write_to_lcd_P(PSTR("J33 "));
				write_to_lcd_P(PSTR("MinT0"));
				write_to_lcd_P(PSTR("\r\n")); // Enter					
			} else if(errorFlag==3) {
				write_to_lcd_P(PSTR("J33 "));
				write_to_lcd_P(PSTR("MinT1"));
				write_to_lcd_P(PSTR("\r\n")); // Enter					
			} else if(errorFlag==4) {
				write_to_lcd_P(PSTR("J33 "));
				write_to_lcd_P(PSTR("MaxT0"));
				write_to_lcd_P(PSTR("\r\n")); // Enter					
			} else if(errorFlag==5) {
				write_to_lcd_P(PSTR("J33 "));
				write_to_lcd_P(PSTR("MaxT1"));
				write_to_lcd_P(PSTR("\r\n")); // Enter					
			} else if(errorFlag==6) {
				write_to_lcd_P(PSTR("J33 "));
				write_to_lcd_P(PSTR("Killed"));
				write_to_lcd_P(PSTR("\r\n")); // Enter					
			}                                 
		break;
       
		case 34: // A34 bed grid write
			#ifdef AUTO_BED_LEVELING_BILINEAR
			if(TFTcode_seen(TFTcmdbuffer, 'X')) x=constrain(TFTcode_value(),0,GRID_MAX_POINTS_X);
			if(TFTcode_seen(TFTcmdbuffer, 'Y')) y=constrain(TFTcode_value(),0,GRID_MAX_POINTS_Y);
			if(TFTcode_seen(TFTcmdbuffer, 'V')) {
				//float i = constrain(TFTcode_value()/100,-10,10);
				z_values[x][y] = (float)constrain(TFTcode_value()/100,-10,10);
				refresh_bed_level();
			}
			if(TFTcode_seen(TFTcmdbuffer, 'S')) {
				refresh_bed_level();
				set_bed_leveling_enabled(true);
				settings.save();
			}
			if(TFTcode_seen(TFTcmdbuffer, 'C')) {
				restore_z_values();
				zprobe_zoffset = SAVE_zprobe_zoffset;
				set_bed_leveling_enabled(true);
				refresh_bed_level();
			}
			#endif
		break;
		
		default: 
		break;
    }   
	}       
}

int z_values_index;
int z_values_size;

void restore_z_values() {
	uint16_t size  = z_values_size;
	int      pos   = z_values_index;
	uint8_t* value = (uint8_t*)&z_values;
	do {
		uint8_t c = eeprom_read_byte((unsigned char*)pos);
		*value = c;
		pos++;
		value++;		
	} while (--size);
}

void mybeep(int beepP,int beepS) {
  if (beepS > 0) {
    #if BEEPER_PIN > 0
      tone(BEEPER_PIN, beepS);
      delay(beepP);
      noTone(BEEPER_PIN);
    #elif defined(ULTRALCD)
      lcd_buzz(beepS, beepP);
    #elif defined(LCD_USE_I2C_BUZZER)
      lcd_buzz(beepP, beepS);
    #endif
  }
  else {
    delay(beepP);
  }
}

void Endstopsbeep() {
	static char last_status = ((READ(X_MIN_PIN)<<3) | (READ(Y_MIN_PIN)<<2) | (READ(Z_MAX_PIN)<<1) | READ(Z_MIN_PIN));
	static unsigned char now_status,status_flag = false, counter = 0;
	now_status=((READ(X_MIN_PIN)<<3) | (READ(Y_MIN_PIN)<<2) | (READ(Z_MAX_PIN)<<1) | READ(Z_MIN_PIN)) & 0xff;
	if(now_status<last_status) {
		counter++;
		if(counter >= 250) {
			counter = 0;
			mybeep(60,2000);
			last_status = now_status;
		}
	} else if(now_status!=last_status) {
	  counter = 0;
	  last_status = now_status;
	}  
}

void setupSDCARD() {
	#if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
		SET_INPUT(SD_DETECT_PIN);
		WRITE(SD_DETECT_PIN, HIGH);
		lcd_sd_status = 2;
		_delay_ms(300);
		card.initsd();
	#endif
}


void setupMyZoffset() {
	#ifdef AUTO_BED_LEVELING_BILINEAR
	SERIAL_ECHOPAIR("MEANL_L:", 0x55);
	SAVE_zprobe_zoffset = zprobe_zoffset;
	#else
	SERIAL_ECHOPAIR("MEANL_L:", 0xaa);
	zprobe_zoffset     = Z_PROBE_OFFSET_FROM_EXTRUDER;
	#endif
    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++) {
	    for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
			if (z_values[x][y] == (float)0.0) {
				z_values[x][y] = (float)-3.5;
			}
		}
    };	
}

#endif // CHIRON_LCD
