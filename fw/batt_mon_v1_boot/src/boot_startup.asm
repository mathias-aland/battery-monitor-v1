$NOMOD51

#include "boot.h"

#define BUF_LOC	0x08

	NAME    BOOT_STARTUP

	PUBLIC  boot_otp

; Declare and locate all memory segments used by the bootloader
?BL_START   	SEGMENT CODE AT BL_START_ADDRESS
?BL_RSVD    	SEGMENT CODE AT BL_LOCK_ADDRESS-2
?BL_STACK   	SEGMENT IDATA
?BUFFER     	SEGMENT DATA AT BUF_LOC

; Create idata segment for stack
	RSEG    ?BL_STACK
	DS      32

; Create data segment for buffer
	RSEG    ?BUFFER
	DS      PKT_MAXLEN+1

; Bootloader entry point (boot_vector)
	RSEG    ?BL_START
?C_STARTUP:
	USING   0

    ; Port 0 setup
	MOV		P0, #0F0H
	MOV     P0MDOUT,#050H
	; Port 1 setup
	MOV		P1, #0

	; CRC0 setup
	ORL		CRC0CN0, #04H	; set ones

	MOV     A,RSTSRC
	CJNE    A,#010H,pin_test	; if not software reset, check pin
	MOV     A,R0
	XRL     A,#BL_SIGNATURE
	JZ      boot_start			; if R0 == signature, start bootloader

app_start:
; check app CRC
	ORL		CRC0CN0, #09H	; init, set pointer to MSB
	MOV     DPTR, #0			; Start address = 0x0000
	CLR     A
	MOVC    A,@A+DPTR
	CPL		A
	JZ		boot_start
    ; 0x0000 is not 0xFF, continue with CRC check
CRC_LOOP:
	CLR		A
	MOVC    A,@A+DPTR
	MOV		CRC0IN, A
	INC		DPTR
	MOV		A, #01BH			; CRC16 address high byte = 0x1B
	CJNE	A, DPH, CRC_LOOP
	MOV		A, #0FEH			; CRC16 address low byte  = 0xFE
	CJNE	A, DPL, CRC_LOOP
; CRC done
	MOV		R0, #2	; 2 CRC bytes
CRC_LOOP2:
	CLR		A
	MOVC    A,@A+DPTR
	CJNE	A, CRC0DAT, boot_start
	INC		DPTR
	DJNZ	R0, CRC_LOOP2
	LJMP    00H		; code OK, start app

; beginning of UART RX routine moved here due to jump length restrictions (saves one ACALL)
UART_RX_STORE:
	MOV		@R0, A		; put into buffer
	INC		R0
	AJMP	MAINLOOP
UART_RX:
	JNB		SCON0_RB8, RX_INIT_WAIT	; framing error
	MOV		A, SBUF0
	JZ		RX_ZERO
	; Non-zero byte received
	; check if there is room in buffer
	CJNE	R0, #?BUFFER+PKT_MAXLEN+1, UART_RX_STORE
	AJMP	MAINLOOP	; buffer full

SOFTRESET:
	MOV		R0, #0
	MOV		RSTSRC, #012h	; software reset

pin_test:
	ANL     A,#03H                  ; A = RSTSRC
	JZ      app_start               ; POR or PINR only
	MOV     R0,#255					; total 583 us
pin_test_loop:                      ; deglitch loop
	JB      BL_START_PIN,app_start  ; +3
	DJNZ    R0,pin_test_loop        ; +4 = 7 cycles per loop

boot_start:

; Disable watchdog
	MOV	    WDTCN,#0DEh
	MOV     WDTCN,#0ADh

	MOV     SP, #?BL_STACK-1	; Stack setup

; MCU is clocked from HFOSC/8 by default
	ORL		LFO0CN, #082H		; Start LF oscillator

; Crossbar setup
	MOV     XBR0,#01H
	MOV     XBR2,#040H

; Timer0/1 setup
	MOV     TMOD,#021H
	MOV     CKCON0,#01H
	MOV		TH1, #060H	; 2392 baud
	MOV		TL1, #060H	; 2392 baud
	SETB    TCON_TR1

; Clear sequence counter (R4), key registers (R6-R7)
	CLR		A
	MOV		R4, A
	MOV		R6, A
	MOV		R7, A

; Set address
	MOV		DPTR, #CAL_START_ADDRESS	; address at first byte of cal page
	MOVC	A, @A+DPTR
	MOV		R2, A
	CPL		A
	JNZ		RX_INIT_WAIT
	MOV		R2, A	; set to 0 (invalid)

RX_INIT_WAIT:
	ACALL	RX_WAITIDLE
RX_INIT:
	SETB	SCON0_REN	; Enable UART
	SETB	P0_B6		; LED on
RX_RESET:
	CLR		SCON0_RI	; clear UART IRQ
	MOV		R0,#?BUFFER	; load buffer pointer
MAINLOOP:
	; timeout init and reset
	; ~86 msec at HFOSC/8 (SCA /4)
	CLR     TCON_TR0
	CLR		A
	MOV		TL0, A
	MOV		TH0, A
	CLR     TCON_TF0
	SETB    TCON_TR0	; Start timeout counter
MAINLOOP_0:
	; Check if soft reset is pending
	JBC		PSW_F1, SOFTRESET
MAINLOOP_1:
	JBC		SCON0_RI, UART_RX	 ; check if data is available in UART
	JNB		TCON_TF0, MAINLOOP_1 ; check if timeout elapsed
	; Timeout, enter low power mode
	CLR		SCON0_REN		; Disable UART
	CLR		P0_B6			; LED off
	MOV		CLKSEL, #02H	; switch to LFOSC
	JB		P0_B5, $ 		; wait for P0.5 to go low (UART RX)
	MOV		CLKSEL,#030H	; Clock config (HFOSC, div by 8)
	JNB		P0_B5, $ 		; wait for P0.5 to go high (UART RX)
	AJMP	RX_INIT

RX_ZERO:
	MOV		A, #256-(PKT_MINLEN+1)-BUF_LOC
	ADD		A, R0
	JNC		RX_RESET
RX_PKT:
	; complete packet received
	; calculate and store packet length
	ADD		A, #PKT_MINLEN
	CJNE	A, ?BUFFER+PKT_INDEX_LEN, RX_RESET	; check if packet length field matches actual received bytes count
	MOV		R3, A	; save received packet length for later use

	; yes, proceed
	; init CRC
	ORL		CRC0CN0, #09H	; init, set pointer to MSB
	MOV		A, R0
	DEC		A
	MOV		R6, A			; store last byte pointer
	MOV		R0, #?BUFFER	; load buffer pointer
	MOV		R1, #?BUFFER	; load COBS pointer

RX_PKT_1:
	MOV		A, R1
	CJNE	A, AR0, RX_PKT_2
	; COBS pointer
	; store new offset
	ADD		A, @R0
	JC		RX_RESET	; COBS pointer value too high
	MOV		R1, A		; store new pointer
	MOV		A, R6
	INC		A
	SUBB	A, R1
	JC		RX_RESET	; COBS pointer value too high
	MOV		@R0, #0		; replace with zero in buffer

RX_PKT_2:
	CJNE	R0, #?BUFFER, RX_PKT_CRC	; check if this is the first byte in the packet
	; yes, this is the first byte
RX_PKT_CONT:
	INC		R0
	AJMP	RX_PKT_1	; continue decoding
RX_PKT_CRC:
	MOV		A, R6					; load last byte pointer
	SUBB	A, R0					; last byte ptr - current ptr
	SUBB	A, #2
	JNC		RX_PKT_NOT_CRC
	; current ptr >= (last byte ptr - 1), check CRC
	MOV		A, CRC0DAT	; load CRC result from module
	XRL		A, @R0		; Compare CRC
	JNZ		RX_RESET	; CRC mismatch, restart RX
	AJMP	RX_PKT_CHKPTR
RX_PKT_NOT_CRC:
	MOV		CRC0IN, @R0					; not first byte, not CRC byte, feed CRC
RX_PKT_CHKPTR:
	MOV		A, R6
	XRL		A, R0
	JNZ		RX_PKT_CONT  ; jump if not all bytes are decoded
RX_PKT_OK:
; Check address
	MOV		A, ?BUFFER+PKT_INDEX_DST
	JZ		RX_RESET	; 0 is not a valid address
	JB		BL_START_PIN, RX_PKT_OK_1 ; check if BL_START_PIN is low (jumper present)
	CJNE	A, #BL_DEFAULT_ADDR, RX_PKT_OK_1
	AJMP	RX_PKT_OK_2
RX_PKT_OK_1:
	CJNE	A, AR2, RX_RESET
RX_PKT_OK_2:
; check if this is a bootloader cmd
	MOV		R1, #?BUFFER+PKT_INDEX_CMD
	MOV		A, @R1
	ANL		A, #0C3H	; keep uppermost 2 bits (CMD MSb) and lowermost 2 bits (flags)
	CJNE	A, #0C0H, RX_RESET	; invalid CMD if not 0x30-0x3F
; Set reply flag
	INC		@R1 ; cmd (0x00) -> reply DUP (0x01)
; Load PKT_INDEX_LEN to R0 for indirect use (will save some bytes of code space)
	MOV		R0, #?BUFFER+PKT_INDEX_LEN
; Set default pkt len
	MOV		@R0, #PKT_MINLEN
; Check for duplicate packet
	MOV		A, ?BUFFER+PKT_INDEX_SEQ
	XRL		A, R4
	JZ		SEND_REPLY
; not duplicate packet
	MOV		R4, ?BUFFER+PKT_INDEX_SEQ
	INC		@R1	; reply DUP (0x01) -> reply ERR (0x02)
; Process command
	; reply ERR flag (0x02) will be set in @R1
	CJNE	@R1, #((BL_CMD_IDENT<<2)|2), CHECK_BL_CMD_RESET
	; identify
	INC		@R0
	MOV		?BUFFER+PKT_INDEX_DATA, #BL_REVISION
	AJMP	SEND_REPLY_OK
CHECK_BL_CMD_RESET:
	CJNE	@R1, #((BL_CMD_RESET<<2)|2), CHECK_BL_CMD_ERASE
	; Soft reset
	SETB	PSW_F1			; Reset pending flag = 1
	AJMP	SEND_REPLY_OK
CHECK_BL_CMD_ERASE:
	CJNE	@R1, #((BL_CMD_ERASE<<2)|2), CHECK_BL_CMD_READ
; BL_CMD_ERASE
	CJNE	R3, #(PKT_MINLEN+3), SEND_REPLY	; check that we have a large enough packet
	INC		@R0			; leave flash address in reply
	ACALL	RW_INIT		; load keys and address
	JC		SEND_REPLY	; init failed if carry == 1
	ACALL	FLASH_ERASEBYTE ; Erase page
	AJMP	SEND_REPLY_OK
CHECK_BL_CMD_READ:
	CJNE	@R1, #((BL_CMD_READ<<2)|2), CHECK_BL_CMD_WRITE
	ACALL	RW_INIT_NOKEYS
	JC		SEND_REPLY	; init failed if carry == 1
	MOV		@R0, #(PKT_MINLEN+65)	; 64 bytes + flash address
	MOV		R0, #?BUFFER+PKT_INDEX_DATA+1	; load data start address to R0
READ_LOOP:
	CLR		A
	MOVC    A,@A+DPTR
	MOV		@R0, A			; put into buffer
	INC		DPTR			; increase flash ptr
	INC		R0				; increase buffer ptr
	DJNZ 	R3, READ_LOOP	; decrease loop counter, check if zero
SEND_REPLY_OK:
	INC		@R1	; reply ERR (0x02) -> reply OK (0x03)
SEND_REPLY:
; encode and send reply
	MOV		R7, ?BUFFER+PKT_INDEX_LEN	; load pkt len to R7
	MOV		R0, #?BUFFER+1	; init read ptr
	MOV		R1, #?BUFFER		; init cobs ptr
; CRC init
	ORL		CRC0CN0, #09H	; init, set pointer to MSB
; run COBS encoder
	MOV		R6, #1	; reset COBS counter
COBS_ENC_LOOP:
	; check if data or CRC byte
	MOV		A, R7
	ADD		A, #(255-2)
	JC		COBS_ENC_CRC
	; this is a CRC byte
	MOV		@R0, CRC0DAT	; update CRC in buffer before encoding
	AJMP	COBS_ENC_NOCRC
COBS_ENC_CRC:
	; not a CRC byte
	MOV		CRC0IN, @R0	; update CRC
COBS_ENC_NOCRC:
	MOV		A, @R0	; load byte from buffer
	JNZ		COBS_ENC_NZ
	; byte is zero, update COBS index
	MOV		@R1, AR6
	MOV		R6, #1	; reset COBS counter
	MOV		R1, AR0	; update COBS pointer
	AJMP	COBS_ENC_NEXT
COBS_ENC_NZ:
	INC		R6		; increase COBS counter
COBS_ENC_NEXT:
	INC		R0		; increase read ptr
	DJNZ	R7, COBS_ENC_LOOP
; final COBS header
	MOV		@R1, AR6
; COBS encoding done
	ACALL	RX_WAITIDLE	; add delay before TX
	MOV		R7, ?BUFFER+PKT_INDEX_LEN	; load pkt len to R7
	INC		R7	; extra byte for COBS header
	MOV		R0, #?BUFFER					; init buffer ptr
	ACALL	TX_ZEROBYTE	; frame start
TX_LOOP:
	MOV		SBUF0, @R0	; data tx
	ACALL	TX_WAIT		; wait for TX
	INC		R0
	DJNZ	R7, TX_LOOP
	ACALL	TX_ZEROBYTE	; frame end
	AJMP	RX_RESET	; restart RX
TX_ZEROBYTE:
	MOV		SBUF0, #0	; frame end
TX_WAIT:
	CLR		SCON0_TI
	JNB		SCON0_TI, $	; wait for TX
	RET

CHECK_BL_CMD_WRITE:
	CJNE	@R1, #((BL_CMD_WRITE<<2)|2), SEND_REPLY
	CJNE	R3, #(PKT_MINLEN+64+3), SEND_REPLY	; check that we have a large enough packet
	ACALL	RW_INIT
	JC		SEND_REPLY
	INC		@R0	; leave flash address in reply
	MOV		R0, #?BUFFER+PKT_INDEX_DATA+3	; load data start address to R0
WRITE_LOOP:
	CLR		A
	MOVC    A,@A+DPTR		; read byte from flash
	CPL		A
	JNZ		SEND_REPLY		; fail if not erased (0xFF)
	ACALL	FLASH_WRITEBYTE	; write flash byte (byte to be written in @R0)
	INC		DPTR			; increase flash ptr
	INC		R0				; increase buffer ptr
	DJNZ 	R3, WRITE_LOOP	; decrease loop counter, check if zero
	AJMP	SEND_REPLY_OK

RW_INIT_FAIL:
	SETB	C	; key error or out of range, carry == 1
	RET
RW_INIT:
	; check keys
	CLR		A
	MOV		R7, ?BUFFER+PKT_INDEX_DATA+1	; key 1 (inverted)
	MOV		?BUFFER+PKT_INDEX_DATA+1, A	; delete key from buffer
	CJNE	R7, #05AH, RW_INIT_FAIL
	MOV		R6, ?BUFFER+PKT_INDEX_DATA+2	; key 2 (inverted)
	MOV		?BUFFER+PKT_INDEX_DATA+2, A	; delete key from buffer
	CJNE	R6, #00EH, RW_INIT_FAIL
RW_INIT_NOKEYS:
	MOV		A, ?BUFFER+PKT_INDEX_DATA
	; Check for out-of-range address
	ADD		A, #(0xFF - (CAL_START_ADDRESS / 64) + 1)
	JC		RW_INIT_FAIL	; jump if out of range (carry bit set)
	MOV		A, ?BUFFER+PKT_INDEX_DATA
	MOV		R3, #64			; init loop counter = 64 bytes
	MOV		B, R3			;
	MUL		AB				; multiply by 64
	MOV		DPL, A			; set DPL to LSB
	MOV		DPH, B			; set DPH to MSB
	; ok, carry == 0 (already cleared by MUL instruction)
	RET

; need total of 15312 cycles for 5 ms
; loop count 2552 at 6 cycles per loop
RX_WAITIDLE:
	JNB		P0_B5, $ 	; wait for P0.5 to go high (UART RX)
	MOV		R7, #10		; 10 * 1534 + 1 = 15341 cycles = 5.0093 ms (~12 bits at 2400 baud)
RX_WAITIDLE1:
	MOV		R6, #255	; 255 * 6 + 4 = 1534 cycles
RX_WAITIDLE2:
	; reset counter if UART RX goes low again
	JNB		P0_B5, RX_WAITIDLE
	DJNZ	R6, RX_WAITIDLE2
	DJNZ	R7, RX_WAITIDLE1
	RET

; Erase flash page
; Setup DPTR before calling
FLASH_ERASEBYTE:
	ORL     PSCTL,#02H	; Set PSEE

; Write byte to flash
; Setup DPTR before calling
; byte to be written in A
FLASH_WRITEBYTE:
	MOV		A, R7				; load key 1
	CPL		A
	MOV		FLKEY, A
	MOV		A, R6				; load key 2
	CPL		A
	MOV		FLKEY, A
	MOV		A, @R0				; load byte to be written
	ORL     PSCTL,#01H			; Set PSWE
	MOV		VDM0CN,#080H		; Make sure voltage monitor is enabled
	MOV		RSTSRC,#02H		    ; ... and selected as a reset source
	MOVX    @DPTR,A
	ANL     PSCTL,#0FCH
	RET

; Reserved Bytes (bl_revision, bl_signature, lock_byte)
	RSEG    ?BL_RSVD
boot_rev:
	DB      BL_REVISION
boot_otp:
	DB      BL_SIGNATURE
lock_byte:
	DB      0xFF

	END
