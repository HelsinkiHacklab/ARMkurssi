/*
 * ioexp_defs.h
 *
 *  Created on: 23.4.2020
 *      Author: Kremmen
 *  SX1509 I/O expander register definitions
 */

#ifndef INC_IOEXP_DEFS_H_
#define INC_IOEXP_DEFS_H_

#define	RegInputDisableB	0x00 	//	Input buffer disable register - I/O[15-8] (Bank B)
#define	RegInputDisableA	0x01 	//	Input buffer disable register - I/O[7-0] (Bank A)
#define	RegLongSlewB		0x02 	//	Output buffer long slew register - I/O[15-8] (Bank B)
#define	RegLongSlewA		0x03 	//	Output buffer long slew register - I/O[7-0] (Bank A)
#define	RegLowDriveB		0x04 	//	Output buffer low drive register - I/O[15-8] (Bank B)
#define	RegLowDriveA		0x05 	//	Output buffer low drive register - I/O[7-0] (Bank A)
#define	RegPullUpB			0x06 	//	Pull-up register - I/O[15-8] (Bank B)
#define	RegPullUpA			0x07 	//	Pull-up register - I/O[7-0] (Bank A)
#define	RegPullDownB		0x08 	//	Pull-down register - I/O[15-8] (Bank B)
#define	RegPullDownA		0x09 	//	Pull-down register - I/O[7-0] (Bank A)
#define	RegOpenDrainB		0x0A 	//	Open drain register - I/O[15-8] (Bank B)
#define	RegOpenDrainA		0x0B 	//	Open drain register - I/O[7-0] (Bank A)
#define	RegPolarityB		0x0C 	//	Polarity register - I/O[15-8] (Bank B)
#define	RegPolarityA		0x0D 	//	Polarity register - I/O[7-0] (Bank A)
#define	RegDirB				0x0E 	//	Direction register - I/O[15-8] (Bank B)
#define	RegDirA				0x0F 	//	Direction register - I/O[7-0] (Bank A)
#define	RegDataB			0x10 	//	Data register - I/O[15-8] (Bank B)
#define	RegDataA			0x11 	//	Data register - I/O[7-0] (Bank A)
#define	RegInterruptMaskB	0x12 	//	Interrupt mask register - I/O[15-8] (Bank B)
#define	RegInterruptMaskA	0x13 	//	Interrupt mask register - I/O[7-0] (Bank A)
#define	RegSenseHighB		0x14 	//	Sense register for I/O[15:12]
#define	RegSenseLowB		0x15 	//	Sense register for I/O[11:8]
#define	RegSenseHighA		0x16 	//	Sense register for I/O[7:4]
#define	RegSenseLowA		0x17 	//	Sense register for I/O[3:0]
#define	RegInterruptSourceB	0x18 	//	Interrupt source register - I/O[15-8] (Bank B)
#define	RegInterruptSourceA	0x19 	//	Interrupt source register - I/O[7-0] (Bank A)
#define	RegEventStatusB		0x1A 	//	Event status register - I/O[15-8] (Bank B)
#define	RegEventStatusA		0x1B 	//	Event status register - I/O[7-0] (Bank A)
#define	RegLevelShifter1	0x1C 	//	Level shifter register
#define	RegLevelShifter2	0x1D 	//	Level shifter register
#define	RegClock			0x1E 	//	Clock management register
#define	RegMisc				0x1F 	//	Miscellaneous device settings register
#define	RegLEDDriverEnableB	0x20 	//	LED driver enable register - I/O[15-8] (Bank B)
#define	RegLEDDriverEnableA	0x21 	//	LED driver enable register - I/O[7-0] (Bank A)

#define	RegDebounceConfig	0x22 	//	Debounce configuration register
#define	RegDebounceEnableB	0x23 	//	Debounce enable register - I/O[15-8] (Bank B)
#define	RegDebounceEnableA	0x24 	//	Debounce enable register - I/O[7-0] (Bank A)
#define	RegKeyConfig1		0x25 	//	Key scan configuration register
#define	RegKeyConfig2		0x26 	//	Key scan configuration register
#define	RegKeyData1			0x27 	//	Key value (column)
#define	RegKeyData2			0x28 	//	Key value (row)

#define	RegTOn				0x29 	//	ON time register for I/O[0]
#define	RegIOn0				0x2A 	//	ON intensity register for I/O[0]
#define	RegOff0				0x2B 	//	OFF time/intensity register for I/O[0]
#define	RegTOn1				0x2C 	//	ON time register for I/O[1]
#define	RegIOn1				0x2D 	//	ON intensity register for I/O[1]
#define	RegOff1				0x2E 	//	OFF time/intensity register for I/O[1]
#define	RegTOn2				0x2F 	//	ON time register for I/O[2]
#define	RegIOn2				0x30 	//	ON intensity register for I/O[2]
#define	RegOff2				0x31 	//	OFF time/intensity register for I/O[2]
#define	RegTOn3				0x32 	//	ON time register for I/O[3]
#define	RegIOn3				0x33 	//	ON intensity register for I/O[3]
#define	RegOff3				0x34 	//	OFF time/intensity register for I/O[3]
#define	RegTOn4				0x35 	//	ON time register for I/O[4]
#define	RegIOn4				0x36 	//	ON intensity register for I/O[4]
#define	RegOff4				0x37 	//	OFF time/intensity register for I/O[4]
#define	RegTRise4			0x38 	//	Fade in register for I/O[4]
#define	RegTFall4			0x39 	//	Fade out register for I/O[4]
#define	RegTOn5				0x3A 	//	ON time register for I/O[5]
#define	RegIOn5				0x3B 	//	ON intensity register for I/O[5]
#define	RegOff5				0x3C 	//	OFF time/intensity register for I/O[5]
#define	RegTRise5			0x3D 	//	Fade in register for I/O[5]
#define	RegTFall5			0x3E 	//	Fade out register for I/O[5]
#define	RegTOn6				0x3F 	//	ON time register for I/O[6]
#define	RegIOn6				0x40 	//	ON intensity register for I/O[6]
#define	RegOff6 	  		0x41 	//	OFF time/intensity register for I/O[6]
#define	RegTRise6 	  		0x42 	//	Fade in register for I/O[6]
#define	RegTFall6 	  		0x43 	//	Fade out register for I/O[6]
#define	RegTOn7 	  		0x44 	//	ON time register for I/O[7]
#define	RegIOn7 	  		0x45 	//	ON intensity register for I/O[7]
#define	RegOff7 	  		0x46 	//	OFF time/intensity register for I/O[7]
#define	RegTRise7 	  		0x47 	//	Fade in register for I/O[7]
#define	RegTFall7 	  		0x48 	//	Fade out register for I/O[7]
#define	RegTOn8 	  		0x49 	//	ON time register for I/O[8]
#define	RegIOn8 	  		0x4A 	//	ON intensity register for I/O[8]
#define	RegOff8 	  		0x4B 	//	OFF time/intensity register for I/O[8]
#define	RegTOn9 	  		0x4C 	//	ON time register for I/O[9]
#define	RegIOn9 	  		0x4D 	//	ON intensity register for I/O[9]
#define	RegOff9 	  		0x4E 	//	OFF time/intensity register for I/O[9]
#define	RegTOn10 	  		0x4F 	//	ON time register for I/O[10]
#define	RegIOn10 	  		0x50 	//	ON intensity register for I/O[10]
#define	RegOff10 	  		0x51 	//	OFF time/intensity register for I/O[10]
#define	RegTOn11 	  		0x52 	//	ON time register for I/O[11]
#define	RegIOn11 	  		0x53 	//	ON intensity register for I/O[11]
#define	RegOff11 	  		0x54 	//	OFF time/intensity register for I/O[11]
#define	RegTOn12 	  		0x55 	//	ON time register for I/O[12]
#define	RegIOn12 	  		0x56 	//	ON intensity register for I/O[12]
#define	RegOff12 	  		0x57 	//	OFF time/intensity register for I/O[12]
#define	RegTRise12 	  		0x58 	//	Fade in register for I/O[12]
#define	RegTFall12 	  		0x59 	//	Fade out register for I/O[12]
#define	RegTOn13 	  		0x5A 	//	ON time register for I/O[13]
#define	RegIOn13 	  		0x5B 	//	ON intensity register for I/O[13]
#define	RegOff13 	  		0x5C 	//	OFF time/intensity register for I/O[13]
#define	RegTRise13 	  		0x5D 	//	Fade in register for I/O[13]
#define	RegTFall13 	  		0x5E 	//	Fade out register for I/O[13]
#define	RegTOn14 	  		0x5F 	//	ON time register for I/O[14]
#define	RegIOn14 	  		0x60 	//	ON intensity register for I/O[14]
#define	RegOff14 	  		0x61 	//	OFF time/intensity register for I/O[14]
#define	RegTRise14 	  		0x62 	//	Fade in register for I/O[14]
#define	RegTFall14 	  		0x63 	//	Fade out register for I/O[14]
#define	RegTOn15 	  		0x64 	//	ON time register for I/O[15]
#define	RegIOn15 	  		0x65 	//	ON intensity register for I/O[15]
#define	RegOff15 	  		0x66 	//	OFF time/intensity register for I/O[15]
#define	RegTRise15 	  		0x67 	//	Fade in register for I/O[15]
#define	RegTFall15 	  		0x68 	//	Fade out register for I/O[15]

#define	RegHighInputB 		0x69 	//	High input enable register - I/O[15-8] (Bank B)
#define	RegHighInputA 	  	0x6A 	//	High input enable register - I/O[7-0] (Bank A)

#define	RegReset 			0x7D 	//	Software reset register

#define	RegTest1 			0x7E 	//	Test register
#define	RegTest2 	  		0x7F 	//	Test register


#endif /* INC_IOEXP_DEFS_H_ */
