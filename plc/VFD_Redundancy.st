(* =====================================================================
   Catalyst 2026, кейс ABB: резервирование ЧРП, схема на 24 контактора
   Логика ПЛК: блокировки Б1-Б5, перевод двигателя на сеть,
   подключение/подхват двигателя ЧРП, реакция на аварию ЧРП.
   Язык: IEC 61131-3 Structured Text (CODESYS 3.5 / ABB Automation Builder)

   Нумерация двигателей: 1..4 = A1..A4, 5..8 = B1..B4
   Источники:  0 = сеть 50 Гц,  1 = ЧРП 1,  2 = ЧРП 2
   Контакторы: xKD_CMD[d, m]  - контактор двигателя m на выход ЧРП d
                                 (1A1..1B4 при d=1, 2A1..2B4 при d=2)
               xKN_CMD[m]     - контактор двигателя m на сеть (BA1..BB4)

   Коды причин отказа команды (nReason):
     0 = разрешено
     1 = Б1  у двигателя уже замкнут другой контактор
     2 = Б2  у ЧРП уже замкнут контактор другого двигателя
     3 = Б3  выход ЧРП не заблокирован
     4 = Б4  нет 50 Гц / синхронизма (или истекло окно переключения)
     5 = Б5  ЧРП в аварии или его вводной автомат Q отключён
     6 = контактор заблокирован: расхождение команды и обратной связи
         (приваривание или обрыв цепи обратной связи)
   ===================================================================== *)


(* ---------------------------- Типы данных ---------------------------- *)

TYPE E_TrStep :
(
    TR_IDLE      := 0,
    TR_WAIT_SYNC := 10,   (* ждём 50 Гц и синхронизм с сетью          *)
    TR_BLOCK     := 20,   (* блокируем выход ЧРП                       *)
    TR_OPEN_K    := 30,   (* размыкаем контактор ЧРП (1A1 и т.п.)      *)
    TR_CLOSE_N   := 40,   (* замыкаем сетевой контактор (BA1 и т.п.)   *)
    TR_DONE      := 50,
    TR_ERROR     := 99
);
END_TYPE

TYPE E_CdStep :
(
    CD_IDLE    := 0,
    CD_BLOCK   := 10,     (* выход ЧРП заблокирован                    *)
    CD_CLOSE_K := 20,     (* замыкаем контактор ЧРП к двигателю        *)
    CD_ENABLE  := 30,     (* разрешаем работу, подхват на ходу         *)
    CD_DONE    := 40,
    CD_ERROR   := 99
);
END_TYPE

TYPE E_TkStep :         (* подхват двигателя после аварии ЧРП          *)
(
    TK_IDLE     := 0,
    TK_FREE_DRV := 10,    (* исправный ЧРП переводит свой двигатель на сеть *)
    TK_CONNECT  := 20,    (* исправный ЧРП подключается к двигателю отказавшего *)
    TK_DONE     := 30,
    TK_ERROR    := 99
);
END_TYPE

TYPE E_Req :            (* запросы оператора с HMI                     *)
(
    REQ_NONE     := 0,
    REQ_CONNECT  := 1,    (* подключить двигатель к ЧРП и запустить    *)
    REQ_TO_MAINS := 2     (* перевести двигатель с ЧРП на сеть         *)
);
END_TYPE


(* ------------------------------ Константы ---------------------------- *)

VAR_GLOBAL CONSTANT
    N_MOT             : INT  := 8;
    F_MAINS_MIN       : REAL := 49.9;      (* Гц, допуск частоты для Б4    *)
    T_SWITCH          : TIME := T#300MS;   (* время срабатывания контактора *)
    T_BLOCK           : TIME := T#200MS;   (* ожидание блокировки выхода   *)
    T_SYNC_TIMEOUT    : TIME := T#30S;     (* ожидание синхронизма         *)
    T_TRANSFER_WINDOW : TIME := T#250MS;   (* окно Б4: от блокировки выхода
                                              до замыкания сетевого
                                              контактора. Дольше - фаза
                                              двигателя уходит от сети     *)
    T_MISMATCH        : TIME := T#500MS;   (* расхождение команда/ОС       *)
END_VAR


(* ---------------------------- Входы/выходы --------------------------- *)

VAR_GLOBAL
    (* --- Входы: вспомогательные контакты (замкнут = TRUE) --- *)
    xKD_FB          : ARRAY[1..2, 1..8] OF BOOL;  (* 1A1..1B4, 2A1..2B4  *)
    xKN_FB          : ARRAY[1..8] OF BOOL;        (* BA1..BB4            *)
    xQ_FB           : ARRAY[1..2] OF BOOL;        (* Q1, Q2 включены     *)
    xF_OK           : ARRAY[1..8] OF BOOL;        (* защита FA/FB в норме *)
    xMotorStandstill: ARRAY[1..8] OF BOOL;        (* реле нулевой скорости *)

    (* --- Входы: статус ЧРП по промышленной сети (Profinet/Modbus TCP) --- *)
    xVfdFault       : ARRAY[1..2] OF BOOL;        (* авария ЧРП          *)
    xVfdOutBlocked  : ARRAY[1..2] OF BOOL;        (* модуляция IGBT снята *)
    xVfdSync        : ARRAY[1..2] OF BOOL;        (* синхронизирован с сетью *)
    rVfdFreq        : ARRAY[1..2] OF REAL;        (* выходная частота, Гц *)

    (* --- Выходы --- *)
    xKD_CMD         : ARRAY[1..2, 1..8] OF BOOL;  (* катушки 1A1..2B4    *)
    xKN_CMD         : ARRAY[1..8] OF BOOL;        (* катушки BA1..BB4    *)
    xQ_TRIP         : ARRAY[1..2] OF BOOL;        (* независимый расцепитель Q *)
    xVfdRunEnable   : ARRAY[1..2] OF BOOL;        (* разрешение модуляции *)
    xVfdFlyingStart : ARRAY[1..2] OF BOOL;        (* подхват на ходу     *)
    rVfdFreqRef     : ARRAY[1..2] OF REAL;        (* задание частоты, Гц *)
    xVfdSyncRequest : ARRAY[1..2] OF BOOL;        (* выйти на 50 Гц и синхронизироваться *)

    (* --- Внутреннее состояние, видно на HMI --- *)
    xSyncWindow     : ARRAY[1..8] OF BOOL;        (* окно Б4 открыто      *)
    xLockout        : ARRAY[1..8] OF BOOL;        (* двигатель заблокирован (код 6) *)
    xVfdFaultLatched: ARRAY[1..2] OF BOOL;        (* авария ЧРП зафиксирована (Б5) *)
    nOnDrive        : ARRAY[1..2] OF INT;         (* какой двигатель на ЧРП, 0 = никакой *)
    xAlarmInvariant : BOOL;                       (* нарушение Б1/Б2 по ОС *)
    nLastReason     : INT;                        (* последняя причина отказа *)

    (* --- Запросы оператора с HMI --- *)
    eReq            : E_Req;
    nReqMotor       : INT;
    nReqDrive       : INT;
    xCmdFaultReset  : ARRAY[1..2] OF BOOL;
END_VAR


(* ======================================================================
   F_PermitClose: можно ли замкнуть контактор двигателя nMotor
   на источник nSrc (0 = сеть, 1/2 = ЧРП). Возвращает код причины.
   ====================================================================== *)
FUNCTION F_PermitClose : INT
VAR_INPUT
    nSrc   : INT;
    nMotor : INT;
END_VAR
VAR
    i, d : INT;
END_VAR

F_PermitClose := 0;

(* Код 6: контактор этого двигателя в расхождении, двигатель заблокирован *)
IF xLockout[nMotor] THEN
    F_PermitClose := 6; RETURN;
END_IF

(* Б5: к отказавшему ЧРП или к ЧРП с отключённым Q подключаться нельзя *)
IF nSrc > 0 THEN
    IF xVfdFaultLatched[nSrc] OR NOT xQ_FB[nSrc] THEN
        F_PermitClose := 5; RETURN;
    END_IF
END_IF

(* Б1: у двигателя замкнут только один из трёх контакторов.
   Проверяем и обратную связь, и команду: команда могла уйти,
   а контактор ещё не успел сработать. *)
FOR d := 1 TO 2 DO
    IF d <> nSrc AND (xKD_FB[d, nMotor] OR xKD_CMD[d, nMotor]) THEN
        F_PermitClose := 1; RETURN;
    END_IF
END_FOR
IF nSrc <> 0 AND (xKN_FB[nMotor] OR xKN_CMD[nMotor]) THEN
    F_PermitClose := 1; RETURN;
END_IF

IF nSrc > 0 THEN
    (* Б2: у ЧРП замкнут только один контактор из восьми *)
    FOR i := 1 TO N_MOT DO
        IF i <> nMotor AND (xKD_FB[nSrc, i] OR xKD_CMD[nSrc, i]) THEN
            F_PermitClose := 2; RETURN;
        END_IF
    END_FOR
    (* Б3: контактор ЧРП переключается только без тока *)
    IF xVfdRunEnable[nSrc] OR NOT xVfdOutBlocked[nSrc] THEN
        F_PermitClose := 3; RETURN;
    END_IF
ELSE
    (* Б4: на сеть - только из окна синхронного переключения,
       либо двигатель стоит (прямой пуск от сети разрешён) *)
    IF NOT (xSyncWindow[nMotor] OR xMotorStandstill[nMotor]) THEN
        F_PermitClose := 4; RETURN;
    END_IF
    (* защита двигателя на сети должна быть в норме *)
    IF NOT xF_OK[nMotor] THEN
        F_PermitClose := 6; RETURN;
    END_IF
END_IF
END_FUNCTION


(* ======================================================================
   F_PermitOpen: размыкать контактор ЧРП можно только без тока (Б3).
   Сетевой контактор можно разомкнуть всегда (двигатель уходит в выбег).
   ====================================================================== *)
FUNCTION F_PermitOpen : INT
VAR_INPUT
    nSrc   : INT;
    nMotor : INT;
END_VAR

F_PermitOpen := 0;
IF nSrc > 0 THEN
    IF xVfdRunEnable[nSrc] OR NOT xVfdOutBlocked[nSrc] THEN
        F_PermitOpen := 3;
    END_IF
END_IF
END_FUNCTION


(* ======================================================================
   FB_TransferToMains: перевод двигателя с ЧРП на сеть на ходу.
   50 Гц + синхронизм -> блокировка выхода -> размыкание 1A1
   -> подтверждение -> замыкание BA1 в окне Б4 -> ЧРП свободен.
   ====================================================================== *)
FUNCTION_BLOCK FB_TransferToMains
VAR_INPUT
    xExecute : BOOL;
    nMotor   : INT;
    nDrive   : INT;
END_VAR
VAR_OUTPUT
    xBusy  : BOOL;
    xDone  : BOOL;
    xError : BOOL;
    nErr   : INT;      (* код причины, 10 = двигатель не на этом ЧРП, 11 = таймаут *)
END_VAR
VAR
    eStep     : E_TrStep;
    rtExec    : R_TRIG;
    tonStep   : TON;
    tonWindow : TON;
    nPerm     : INT;
    eLastStep : E_TrStep;
    xWinOwned : BOOL;     (* этот блок открыл окно Б4 для nMotor        *)
    xTouched  : BOOL;     (* блок уже заблокировал выход ЧРП            *)
END_VAR

rtExec(CLK := xExecute);

(* таймер шага перезапускается при каждой смене шага *)
tonStep(IN := (eStep = eLastStep) AND (eStep <> TR_IDLE), PT := T_SYNC_TIMEOUT);
eLastStep := eStep;

(* окно Б4: открыто от блокировки выхода до замыкания сетевого контактора,
   но не дольше T_TRANSFER_WINDOW. Пишем флаг только для своего двигателя. *)
tonWindow(IN := (eStep = TR_OPEN_K) OR (eStep = TR_CLOSE_N), PT := T_TRANSFER_WINDOW);
IF (eStep = TR_OPEN_K) OR (eStep = TR_CLOSE_N) THEN
    xSyncWindow[nMotor] := NOT tonWindow.Q;
    xWinOwned := TRUE;
ELSIF xWinOwned THEN
    xSyncWindow[nMotor] := FALSE;
    xWinOwned := FALSE;
END_IF

CASE eStep OF

TR_IDLE:
    xDone := FALSE; xBusy := FALSE;
    IF rtExec.Q THEN
        xError := FALSE; nErr := 0; xTouched := FALSE;
        IF NOT xKD_FB[nDrive, nMotor] THEN
            nErr := 10; xError := TRUE;        (* двигатель не на этом ЧРП: ничего не трогаем *)
        ELSE
            xBusy := TRUE;
            xVfdSyncRequest[nDrive] := TRUE;   (* ЧРП выходит на 50 Гц и ловит фазу *)
            eStep := TR_WAIT_SYNC;
        END_IF
    END_IF

TR_WAIT_SYNC:
    IF rVfdFreq[nDrive] >= F_MAINS_MIN AND xVfdSync[nDrive] THEN
        eStep := TR_BLOCK;
    ELSIF tonStep.Q THEN
        nErr := 11; eStep := TR_ERROR;
    END_IF

TR_BLOCK:
    xVfdRunEnable[nDrive] := FALSE;            (* снять импульсы IGBT *)
    xTouched := TRUE;
    IF xVfdOutBlocked[nDrive] THEN
        eStep := TR_OPEN_K;
    ELSIF tonStep.ET > T_BLOCK THEN
        nErr := 3; eStep := TR_ERROR;
    END_IF

TR_OPEN_K:
    nPerm := F_PermitOpen(nSrc := nDrive, nMotor := nMotor);
    IF nPerm <> 0 THEN
        nErr := nPerm; eStep := TR_ERROR;
    ELSE
        xKD_CMD[nDrive, nMotor] := FALSE;
        IF NOT xKD_FB[nDrive, nMotor] THEN
            eStep := TR_CLOSE_N;
        ELSIF tonStep.ET > T_SWITCH THEN
            nErr := 6; eStep := TR_ERROR;      (* контактор приварился *)
        END_IF
    END_IF

TR_CLOSE_N:
    nPerm := F_PermitClose(nSrc := 0, nMotor := nMotor);
    IF nPerm <> 0 THEN
        nErr := nPerm; eStep := TR_ERROR;
    ELSE
        xKN_CMD[nMotor] := TRUE;
        IF xKN_FB[nMotor] THEN
            eStep := TR_DONE;
        ELSIF tonStep.ET > T_SWITCH THEN
            xKN_CMD[nMotor] := FALSE;
            nErr := 6; eStep := TR_ERROR;
        END_IF
    END_IF

TR_DONE:
    xVfdSyncRequest[nDrive] := FALSE;
    xBusy := FALSE; xDone := TRUE;             (* ЧРП свободен для следующего двигателя *)
    IF NOT xExecute THEN xDone := FALSE; eStep := TR_IDLE; END_IF

TR_ERROR:
    (* Если ошибка до блокировки (нет синхронизма) - двигатель остаётся
       на ЧРП и работает дальше. Если после - выход ЧРП заблокирован,
       сетевой контактор не замыкается, двигатель в выбеге,
       оператор видит причину и может подхватить его снова. *)
    IF xTouched THEN xVfdRunEnable[nDrive] := FALSE; END_IF
    xVfdSyncRequest[nDrive] := FALSE;
    xBusy := FALSE; xError := TRUE;
    IF NOT xExecute THEN xError := FALSE; eStep := TR_IDLE; END_IF

END_CASE
END_FUNCTION_BLOCK


(* ======================================================================
   FB_ConnectToDrive: подключить двигатель к ЧРП и запустить.
   Если двигатель вращается - подхват на ходу (flying start).
   Блокировка выхода -> замыкание 1B1 -> подтверждение -> разрешение.
   ====================================================================== *)
FUNCTION_BLOCK FB_ConnectToDrive
VAR_INPUT
    xExecute : BOOL;
    nMotor   : INT;
    nDrive   : INT;
END_VAR
VAR_OUTPUT
    xBusy  : BOOL;
    xDone  : BOOL;
    xError : BOOL;
    nErr   : INT;
END_VAR
VAR
    eStep     : E_CdStep;
    rtExec    : R_TRIG;
    tonStep   : TON;
    eLastStep : E_CdStep;
    nPerm     : INT;
END_VAR

rtExec(CLK := xExecute);
tonStep(IN := (eStep = eLastStep) AND (eStep <> CD_IDLE), PT := T#10S);
eLastStep := eStep;

CASE eStep OF

CD_IDLE:
    xDone := FALSE; xBusy := FALSE;
    IF rtExec.Q THEN
        xError := FALSE; nErr := 0; xBusy := TRUE;
        eStep := CD_BLOCK;
    END_IF

CD_BLOCK:
    xVfdRunEnable[nDrive] := FALSE;
    IF xVfdOutBlocked[nDrive] THEN
        eStep := CD_CLOSE_K;
    ELSIF tonStep.ET > T_BLOCK THEN
        nErr := 3; eStep := CD_ERROR;
    END_IF

CD_CLOSE_K:
    nPerm := F_PermitClose(nSrc := nDrive, nMotor := nMotor);
    IF nPerm <> 0 THEN
        nErr := nPerm; eStep := CD_ERROR;
    ELSE
        xKD_CMD[nDrive, nMotor] := TRUE;
        IF xKD_FB[nDrive, nMotor] THEN
            eStep := CD_ENABLE;
        ELSIF tonStep.ET > T_SWITCH THEN
            xKD_CMD[nDrive, nMotor] := FALSE;
            nErr := 6; eStep := CD_ERROR;
        END_IF
    END_IF

CD_ENABLE:
    (* Б3: работа разрешается только после подтверждения положения *)
    xVfdFlyingStart[nDrive] := NOT xMotorStandstill[nMotor];
    xVfdRunEnable[nDrive] := TRUE;
    IF NOT xVfdOutBlocked[nDrive] THEN
        eStep := CD_DONE;
    ELSIF tonStep.ET > T#2S THEN
        nErr := 3; eStep := CD_ERROR;
    END_IF

CD_DONE:
    xBusy := FALSE; xDone := TRUE;
    IF NOT xExecute THEN xDone := FALSE; eStep := CD_IDLE; END_IF

CD_ERROR:
    xVfdRunEnable[nDrive] := FALSE;
    xBusy := FALSE; xError := TRUE;
    IF NOT xExecute THEN xError := FALSE; eStep := CD_IDLE; END_IF

END_CASE
END_FUNCTION_BLOCK


(* ======================================================================
   PRG_Main: основной цикл (задача 10 мс).
   ====================================================================== *)
PROGRAM PRG_Main
VAR
    d, o, m, nClosed : INT;
    rtFault     : ARRAY[1..2] OF R_TRIG;
    tonMisD     : ARRAY[1..2, 1..8] OF TON;
    tonMisN     : ARRAY[1..8] OF TON;

    fbTransfer  : ARRAY[1..2] OF FB_TransferToMains;
    fbConnect   : ARRAY[1..2] OF FB_ConnectToDrive;
    xTrExec     : ARRAY[1..2] OF BOOL;
    xCdExec     : ARRAY[1..2] OF BOOL;
    nTrMotor, nCdMotor : ARRAY[1..2] OF INT;

    eTk         : E_TkStep;
    nTkFailed   : INT;     (* отказавший ЧРП             *)
    nTkHealthy  : INT;     (* исправный ЧРП              *)
    nTkOrphan   : INT;     (* двигатель отказавшего ЧРП  *)
    nTkBusyMot  : INT;     (* двигатель исправного ЧРП   *)
END_VAR

(* ---- 1. Кто на каком ЧРП (по обратной связи) ---- *)
FOR d := 1 TO 2 DO
    nOnDrive[d] := 0;
    FOR m := 1 TO N_MOT DO
        IF xKD_FB[d, m] THEN nOnDrive[d] := m; END_IF
    END_FOR
END_FOR

(* ---- 2. Контроль контакторов: команда и обратная связь расходятся
          дольше T_MISMATCH -> приваривание/обрыв, двигатель блокируется ---- *)
FOR m := 1 TO N_MOT DO
    FOR d := 1 TO 2 DO
        tonMisD[d, m](IN := xKD_CMD[d, m] XOR xKD_FB[d, m], PT := T_MISMATCH);
        IF tonMisD[d, m].Q THEN xLockout[m] := TRUE; END_IF
    END_FOR
    tonMisN[m](IN := xKN_CMD[m] XOR xKN_FB[m], PT := T_MISMATCH);
    IF tonMisN[m].Q THEN xLockout[m] := TRUE; END_IF
END_FOR

(* ---- 3. Проверка инвариантов Б1/Б2 по фактическому положению ----
   Если по обратной связи замкнуто два контактора - аварийно
   блокируем оба ЧРП (аппаратная блокировка должна была не дать). *)
xAlarmInvariant := FALSE;
FOR m := 1 TO N_MOT DO
    nClosed := BOOL_TO_INT(xKD_FB[1, m]) + BOOL_TO_INT(xKD_FB[2, m]) + BOOL_TO_INT(xKN_FB[m]);
    IF nClosed > 1 THEN xAlarmInvariant := TRUE; END_IF
END_FOR
FOR d := 1 TO 2 DO
    nClosed := 0;
    FOR m := 1 TO N_MOT DO
        nClosed := nClosed + BOOL_TO_INT(xKD_FB[d, m]);
    END_FOR
    IF nClosed > 1 THEN xAlarmInvariant := TRUE; END_IF
END_FOR

(* ---- 4. Б5: авария ЧРП ---- *)
FOR d := 1 TO 2 DO
    rtFault[d](CLK := xVfdFault[d]);
    IF rtFault[d].Q AND NOT xVfdFaultLatched[d] THEN
        xVfdFaultLatched[d] := TRUE;
        (* запуск подхвата, если на отказавшем ЧРП был двигатель *)
        IF eTk = TK_IDLE AND nOnDrive[d] > 0 THEN
            nTkFailed  := d;
            nTkHealthy := 3 - d;
            nTkOrphan  := nOnDrive[d];
            nTkBusyMot := nOnDrive[3 - d];
            IF nTkBusyMot > 0 THEN eTk := TK_FREE_DRV; ELSE eTk := TK_CONNECT; END_IF
        END_IF
    END_IF
    (* сброс аварии оператором - только если ЧРП исправен *)
    IF xCmdFaultReset[d] AND NOT xVfdFault[d] THEN
        xVfdFaultLatched[d] := FALSE;
        xQ_TRIP[d] := FALSE;            (* включение Q - отдельной командой *)
    END_IF
END_FOR

(* ---- 5. Подхват двигателя исправным ЧРП ---- *)
CASE eTk OF
TK_FREE_DRV:     (* исправный ЧРП сначала отдаёт свой двигатель сети *)
    xTrExec[nTkHealthy] := TRUE; nTrMotor[nTkHealthy] := nTkBusyMot;
    IF fbTransfer[nTkHealthy].xDone THEN
        xTrExec[nTkHealthy] := FALSE; eTk := TK_CONNECT;
    ELSIF fbTransfer[nTkHealthy].xError THEN
        xTrExec[nTkHealthy] := FALSE; eTk := TK_ERROR;
    END_IF
TK_CONNECT:      (* блокировка выхода, замыкание 1B1, подхват на ходу *)
    xCdExec[nTkHealthy] := TRUE; nCdMotor[nTkHealthy] := nTkOrphan;
    IF fbConnect[nTkHealthy].xDone THEN
        xCdExec[nTkHealthy] := FALSE; eTk := TK_DONE;
    ELSIF fbConnect[nTkHealthy].xError THEN
        xCdExec[nTkHealthy] := FALSE; eTk := TK_ERROR;
    END_IF
TK_DONE, TK_ERROR:
    eTk := TK_IDLE;   (* результат и код ошибки видны на HMI через fbConnect/fbTransfer *)
END_CASE

(* ---- 6. Запросы оператора (если не идёт подхват) ---- *)
IF eTk = TK_IDLE AND eReq <> REQ_NONE AND nReqDrive >= 1 AND nReqDrive <= 2
   AND nReqMotor >= 1 AND nReqMotor <= N_MOT THEN
    CASE eReq OF
    REQ_CONNECT:
        xCdExec[nReqDrive] := TRUE; nCdMotor[nReqDrive] := nReqMotor;
        IF fbConnect[nReqDrive].xDone OR fbConnect[nReqDrive].xError THEN
            nLastReason := fbConnect[nReqDrive].nErr;
            xCdExec[nReqDrive] := FALSE; eReq := REQ_NONE;
        END_IF
    REQ_TO_MAINS:
        xTrExec[nReqDrive] := TRUE; nTrMotor[nReqDrive] := nReqMotor;
        IF fbTransfer[nReqDrive].xDone OR fbTransfer[nReqDrive].xError THEN
            nLastReason := fbTransfer[nReqDrive].nErr;
            xTrExec[nReqDrive] := FALSE; eReq := REQ_NONE;
        END_IF
    END_CASE
END_IF

(* ---- 7. Вызов функциональных блоков ---- *)
FOR d := 1 TO 2 DO
    fbTransfer[d](xExecute := xTrExec[d], nMotor := MAX(nTrMotor[d], 1), nDrive := d);
    fbConnect[d] (xExecute := xCdExec[d], nMotor := MAX(nCdMotor[d], 1), nDrive := d);
END_FOR

(* ---- 8. Б5: отказавший ЧРП - выход заблокирован, все его контакторы
          и вводной автомат Q отключены. Выполняется ПОСЛЕ блоков,
          чтобы последнее слово в цикле было за безопасностью. ---- *)
FOR d := 1 TO 2 DO
    IF xVfdFaultLatched[d] THEN
        xVfdRunEnable[d] := FALSE;
        xVfdSyncRequest[d] := FALSE;
        FOR m := 1 TO N_MOT DO xKD_CMD[d, m] := FALSE; END_FOR
        xQ_TRIP[d] := TRUE;
    END_IF
END_FOR

(* ---- 8a. Аварийный останов при нарушении инварианта ---- *)
IF xAlarmInvariant THEN
    xVfdRunEnable[1] := FALSE;
    xVfdRunEnable[2] := FALSE;
END_IF

(* ---- 9. Защита двигателя на сети: при срабатывании FA/FB
          сетевой контактор отключается ---- *)
FOR m := 1 TO N_MOT DO
    IF NOT xF_OK[m] THEN xKN_CMD[m] := FALSE; END_IF
END_FOR
END_PROGRAM


(* =====================================================================
   АППАРАТНАЯ ЧАСТЬ (дублирует Б1 и Б2 без ПЛК, в цепях катушек):
   - Б1: в цепь катушки 1A1 последовательно включены НЗ вспомогательные
     контакты 2A1 и BA1; аналогично для 2A1 и BA1. Если ПЛК ошибётся,
     катушка второго контактора физически не получит питание.
   - Б2: в общую цепь катушек 1A1..1B4 включены НЗ контакты остальных
     семи контакторов ЧРП 1 (или цепочка "только один из восьми").
   - Зеркальные контакты (IEC 60947-4-1, приложение F) гарантируют,
     что НЗ контакт не замкнут, пока силовой контакт приварен.
   - STO (Safe Torque Off) ЧРП заведён в контур безопасности: снятие
     импульсов IGBT независимо от программы ЧРП.
   ===================================================================== *)
