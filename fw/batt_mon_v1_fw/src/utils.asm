$NOMOD51

#include "SI_EFM8BB1_Defs.inc"
#include "SI_EFM8BB1_Register_Enums.inc"

NAME    ENTER_BL

?PR?enter_bl?ENTER_BL    SEGMENT CODE
        PUBLIC  enter_bl

        RSEG    ?PR?enter_bl?ENTER_BL
enter_bl:
        USING   0
        MOV		R0, #0A5H		; signature to enter bootloader
        MOV		RSTSRC, #012h	; software reset
        RET

        END
