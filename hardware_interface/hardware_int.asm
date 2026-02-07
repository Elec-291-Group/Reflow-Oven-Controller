;----------------------------------------------------------------;
; UI_FINAL_UNITS.ASM                                             ;
;----------------------------------------------------------------;
$NOLIST
$include(MODMAX10)
$LIST

org 0000H
    ljmp main

; ----------------------------------------------------------------
; 1. PIN DEFINITIONS
; ----------------------------------------------------------------
ELCD_RS equ P1.7 ; Pin 18
ELCD_E  equ P1.1 ; Pin 10
ELCD_D4 equ P0.7 ; Pin 8
ELCD_D5 equ P0.5 ; Pin 6
ELCD_D6 equ P0.3 ; Pin 4
ELCD_D7 equ P0.1 ; Pin 2

BTN_SOAK_TEMP equ P0.0 ; Pin 1
BTN_SOAK_TIME equ P0.2 ; Pin 3
BTN_REFL_TEMP equ P0.4 ; Pin 5
BTN_REFL_TIME equ P0.6 ; Pin 7

ROW1 equ P1.2 ; Pin 13
ROW2 equ P1.4 ; Pin 15
ROW3 equ P1.6 ; Pin 17
ROW4 equ P2.0 ; Pin 19
COL1 equ P2.2 ; Pin 21
COL2 equ P2.4 ; Pin 23
COL3 equ P2.6 ; Pin 25
COL4 equ P3.0 ; Pin 27

; ----------------------------------------------------------------
; 2. VARIABLES & STORAGE
; ----------------------------------------------------------------
dseg at 0x30
Current_State: ds 1   ; 0=Home, 1..4=Modes
Cursor_Idx:    ds 1   ; Tracks cursor position

; --- BUFFER STORAGE ---
; Temp: 3 Digits + 'C' + Null = 5 bytes
Buf_Soak_Temp: ds 5   
; Time: "00:00" + 's' + Null = 7 bytes
Buf_Soak_Time: ds 7   
Buf_Refl_Temp: ds 5   
Buf_Refl_Time: ds 7   

cseg
; --- STRINGS ---
Txt_Home:     db 'Select Mode:    ', 0
Txt_SoakT:    db 'Set Soak Temp   ', 0
Txt_SoakTime: db 'Set Soak Time   ', 0
Txt_ReflT:    db 'Set Reflow Temp ', 0
Txt_ReflTime: db 'Set Reflow Time ', 0

$include(LCD_4bit_DE10Lite_no_RW.inc)

; ----------------------------------------------------------------
; 3. MAIN PROGRAM
; ----------------------------------------------------------------
main:
    mov SP, #0x7F
    
    ; --- PORT CONFIG ---
    mov P0MOD, #0xAA 
    mov P1MOD, #0xD6
    mov P2MOD, #0x01
    mov P3MOD, #0x00

    ; --- INIT LCD ---
    lcall ELCD_4BIT
    mov A, #0x0F ; Blink ON
    lcall ?WriteCommand

    ; --- INITIALIZE BUFFERS ---
    lcall Init_All_Buffers

    ; --- STARTUP ---
    mov Current_State, #0
    lcall Update_Screen_Full
    
    ; Dummy Hex
    mov HEX0, #0x46 
    mov HEX1, #0x92 
    mov HEX2, #0xA4 
    mov HEX3, #0xFF
    mov HEX4, #0xFF
    mov HEX5, #0xFF

Forever:
    lcall Check_Buttons
    lcall Check_Keypad
    ljmp Forever

; ----------------------------------------------------------------
; 4. INITIALIZATION ROUTINES (UPDATED FOR UNITS)
; ----------------------------------------------------------------
Init_All_Buffers:
    ; Init Temps to "C" (Shifting Style)
    mov R0, #Buf_Soak_Temp
    lcall Load_Temp_Init
    mov R0, #Buf_Refl_Temp
    lcall Load_Temp_Init

    ; Init Times to "00:00s" (Fixed Style)
    lcall Reset_Soak_Time
    lcall Reset_Refl_Time
    ret

Load_Temp_Init:
    ; Starts with just "C"
    mov @R0, #'C'
    inc R0
    mov @R0, #0 ; Null
    ret

Reset_Soak_Time:
    mov R0, #Buf_Soak_Time
    lcall Load_Time_Template
    ret

Reset_Refl_Time:
    mov R0, #Buf_Refl_Time
    lcall Load_Time_Template
    ret

Load_Time_Template:
    ; Loads "00:00s" + Null
    mov @R0, #'0'
    inc R0
    mov @R0, #'0'
    inc R0
    mov @R0, #':'
    inc R0
    mov @R0, #'0'
    inc R0
    mov @R0, #'0'
    inc R0
    mov @R0, #'s' ; UNIT ADDED HERE
    inc R0
    mov @R0, #0
    ret

; ----------------------------------------------------------------
; 5. BUTTON HANDLER
; ----------------------------------------------------------------
Check_Buttons:
    jnb BTN_SOAK_TEMP, Btn1_Press
    jnb BTN_SOAK_TIME, Btn2_Press
    jnb BTN_REFL_TEMP, Btn3_Press
    jnb BTN_REFL_TIME, Btn4_Press
    ret

Btn1_Press:
    lcall Wait25ms
    mov Current_State, #1
    mov Cursor_Idx, #0 
    sjmp Redraw
Btn2_Press:
    lcall Wait25ms
    mov Current_State, #2
    mov Cursor_Idx, #0
    sjmp Redraw
Btn3_Press:
    lcall Wait25ms
    mov Current_State, #3
    mov Cursor_Idx, #0
    sjmp Redraw
Btn4_Press:
    lcall Wait25ms
    mov Current_State, #4
    mov Cursor_Idx, #0
    sjmp Redraw

Redraw:
    lcall Update_Screen_Full
    jnb BTN_SOAK_TEMP, $
    jnb BTN_SOAK_TIME, $
    jnb BTN_REFL_TEMP, $
    jnb BTN_REFL_TIME, $
    ret

; ----------------------------------------------------------------
; 6. KEYPAD HANDLER (SHIFTING UNIT LOGIC)
; ----------------------------------------------------------------
Check_Keypad:
    mov A, Current_State
    jz Keypad_Exit 
    lcall Keypad_Scan
    jnc Keypad_Exit

    ; --- CHECK STAR (*) -> RESET ---
    mov A, R7
    cjne A, #14, Check_Hash
    lcall Reset_Current_Buffer
    lcall Update_Screen_Full 
    mov Cursor_Idx, #0 
    ret

    ; --- CHECK HASH (#) -> IGNORE ---
Check_Hash:
    mov A, R7
    cjne A, #12, Check_Number
    ret 

Check_Number:
    mov A, R7
    clr C
    subb A, #10
    jnc Symbol_Key 
    
    mov A, R7
    add A, #0x30 ; ASCII
    mov R5, A 

    ; --- WRITE DIGIT ---
    lcall Get_Current_Buffer_Addr 
    mov A, Cursor_Idx
    add A, R0
    mov R0, A 
    
    mov A, R5
    mov @R0, A ; Overwrite whatever is there (even the 'C')
    
    ; --- ADVANCE CURSOR ---
    inc Cursor_Idx
    
    ; --- LOGIC BRANCH ---
    mov A, Current_State
    cjne A, #1, Check_T_Mode2
    sjmp Handle_Temp_Shift
Check_T_Mode2:
    cjne A, #3, Handle_Time_Skip
    sjmp Handle_Temp_Shift

Handle_Temp_Shift:
    ; === THE MAGNETIC C LOGIC ===
    ; We just wrote a digit. Now write 'C' at the NEW Cursor position.
    ; R0 currently points to where we just wrote.
    inc R0 ; Next slot
    mov @R0, #'C'
    inc R0
    mov @R0, #0 ; Terminate
    
    ; Limit Check: If we hit 3 digits, back up so we don't go off screen
    mov A, Cursor_Idx
    cjne A, #3, Do_Refresh
    dec Cursor_Idx
    sjmp Do_Refresh

Handle_Time_Skip:
    ; Time Logic (00:00s)
    ; Skip Colon (Index 2)
    mov A, Cursor_Idx
    cjne A, #2, Check_Limit_Time
    inc Cursor_Idx 
    sjmp Normal_Time_Update

Check_Limit_Time:
    ; Stop at 5 (don't overwrite 's' at index 5)
    mov A, Cursor_Idx
    cjne A, #5, Normal_Time_Update
    dec Cursor_Idx 
    sjmp Do_Refresh

Normal_Time_Update:
    sjmp Do_Refresh

Do_Refresh:
    lcall Update_Screen_Full
    ret

Symbol_Key:
    ret
Keypad_Exit:
    ret

; ----------------------------------------------------------------
; 7. BUFFER UTILITIES
; ----------------------------------------------------------------
Get_Current_Buffer_Addr:
    mov A, Current_State
    cjne A, #1, G_M2
    mov R0, #Buf_Soak_Temp
    ret
G_M2:
    cjne A, #2, G_M3
    mov R0, #Buf_Soak_Time
    ret
G_M3:
    cjne A, #3, G_M4
    mov R0, #Buf_Refl_Temp
    ret
G_M4:
    mov R0, #Buf_Refl_Time
    ret

Reset_Current_Buffer:
    mov A, Current_State
    cjne A, #1, R_M2
    ; Reset Temp 1 (Init to 'C')
    mov R0, #Buf_Soak_Temp
    lcall Load_Temp_Init
    ret
R_M2:
    cjne A, #2, R_M3
    lcall Reset_Soak_Time
    ret
R_M3:
    cjne A, #3, R_M4
    ; Reset Temp 2 (Init to 'C')
    mov R0, #Buf_Refl_Temp
    lcall Load_Temp_Init
    ret
R_M4:
    lcall Reset_Refl_Time
    ret

; ----------------------------------------------------------------
; 8. SCREEN UTILITIES
; ----------------------------------------------------------------
Update_Screen_Full:
    lcall Clear_Screen_Func 
    
    ; Line 1
    Set_Cursor(1, 1)
    mov A, Current_State
    cjne A, #0, U_S1
    Send_Constant_String(#Txt_Home)
    ret 
U_S1:
    cjne A, #1, U_S2
    Send_Constant_String(#Txt_SoakT)
    sjmp Draw_Buffer
U_S2:
    cjne A, #2, U_S3
    Send_Constant_String(#Txt_SoakTime)
    sjmp Draw_Buffer
U_S3:
    cjne A, #3, U_S4
    Send_Constant_String(#Txt_ReflT)
    sjmp Draw_Buffer
U_S4:
    Send_Constant_String(#Txt_ReflTime)

Draw_Buffer:
    ; Line 2
    Set_Cursor(2, 1)
    lcall Get_Current_Buffer_Addr 
    
Print_RAM_Loop:
    mov A, @R0
    jz Print_Done 
    lcall ?WriteData
    inc R0
    sjmp Print_RAM_Loop
Print_Done:

    ; Restore Cursor
    mov A, Cursor_Idx
    add A, #0xC0
    lcall ?WriteCommand
    ret

Clear_Screen_Func:
    mov A, #0x01 
    lcall ?WriteCommand
    mov R2, #10
    lcall Wait25ms
    mov A, #0x0F 
    lcall ?WriteCommand
    ret

; ----------------------------------------------------------------
; 9. HARDWARE SCANNERS
; ----------------------------------------------------------------
Keypad_Scan:
    clr ROW1
    clr ROW2
    clr ROW3
    clr ROW4
    mov C, COL1
    anl C, COL2
    anl C, COL3
    anl C, COL4
    jnc Keypad_Debounce 
    clr C 
    ret
Keypad_Debounce:
    lcall Wait25ms
    mov C, COL1
    anl C, COL2
    anl C, COL3
    anl C, COL4
    jnc Keypad_Find_Row 
    clr C
    ret
Keypad_Find_Row:
    setb ROW1
    setb ROW2
    setb ROW3
    setb ROW4
    clr ROW1
    jnb COL1, K_1
    jnb COL2, K_2
    jnb COL3, K_3
    jnb COL4, K_A
    setb ROW1
    clr ROW2
    jnb COL1, K_4
    jnb COL2, K_5
    jnb COL3, K_6
    jnb COL4, K_B
    setb ROW2
    clr ROW3
    jnb COL1, K_7
    jnb COL2, K_8
    jnb COL3, K_9
    jnb COL4, K_C
    setb ROW3
    clr ROW4
    jnb COL1, K_Star
    jnb COL2, K_0
    jnb COL3, K_Hash
    jnb COL4, K_D
    setb ROW4
    clr C 
    ret

K_1: mov R7, #1
     sjmp Wait_Rel
K_2: mov R7, #2
     sjmp Wait_Rel
K_3: mov R7, #3
     sjmp Wait_Rel
K_A: mov R7, #10
     sjmp Wait_Rel
K_4: mov R7, #4
     sjmp Wait_Rel
K_5: mov R7, #5
     sjmp Wait_Rel
K_6: mov R7, #6
     sjmp Wait_Rel
K_B: mov R7, #11
     sjmp Wait_Rel
K_7: mov R7, #7
     sjmp Wait_Rel
K_8: mov R7, #8
     sjmp Wait_Rel
K_9: mov R7, #9
     sjmp Wait_Rel
K_C: mov R7, #13
     sjmp Wait_Rel
K_Star: mov R7, #14
     sjmp Wait_Rel
K_0: mov R7, #0
     sjmp Wait_Rel
K_Hash: mov R7, #12
     sjmp Wait_Rel
K_D: mov R7, #15
     sjmp Wait_Rel

Wait_Rel:
    mov C, COL1
    anl C, COL2
    anl C, COL3
    anl C, COL4
    jnc Wait_Rel 
    setb C 
    setb ROW1
    setb ROW2
    setb ROW3
    setb ROW4
    ret

Wait25ms:
    mov R0, #15
W_L3: mov R1, #74
W_L2: mov R2, #250
W_L1: djnz R2, W_L1 
    djnz R1, W_L2 
    djnz R0, W_L3 
    ret

END