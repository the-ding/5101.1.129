;******************************************************************************
; export symbols

        XDEF  udelay
        XDEF  mdelay

; code section
MyCode:     SECTION

udelay:
      PSHD
uloops:
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP
      NOP

      SUBD  #1
      BNE   uloops

      PULD
      RTC


mdelay:
      PSHD
      PSHX
      TFR   D, X
mloops:
      LDD   #997
      CALL  udelay

      SUBX  #1
      BNE   mloops
      
      PULX
      PULD
      RTC
