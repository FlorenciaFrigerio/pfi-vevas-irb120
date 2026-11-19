MODULE VEVAS_RWS
    ! Servidor PC de VEVAS (adaptado de LabJointRws V3, robotica-IMT342). Sin main.
    ! Codigos vvRbCodigo: 0 listo, 1 moviendo, 2 llegada, -1 vacio, -2 limites, -3 paso, -4 modo.
    ! Mantener este archivo por debajo de 4800 caracteres: RWS corta el texto en 5000.
    LOCAL CONST num maxPasoDeg := 45;
    LOCAL CONST num margenDeg := 2;
    LOCAL CONST bool permitirAuto := TRUE;
    LOCAL CONST bool confirmarEnPendant := TRUE;
    LOCAL CONST speeddata vPc := [20,5,20,5];
    PERS jointtarget vvPcTarget := [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS num vvPcSeq := 0;
    PERS num vvPcFin := 0;
    PERS num vvRbSeq := 0;
    PERS num vvRbCodigo := 0;
    PERS num vvRbActivo := 0;

    PROC VevasMostrarObjetivo()
        MostrarEjes vvPcTarget.robax;
    ENDPROC

    PROC VevasServidor()
        VAR jointtarget objetivo;
        VAR jointtarget actual;
        VAR num seq := 0;
        VAR num respuesta := 0;
        VAR errnum motivo := 0;

        vvRbActivo := 0;
        IF ModoPermitido() = FALSE THEN
            TPWrite "Seleccione AUTO o manual reducido.";
            RETURN;
        ENDIF
        IF confirmarEnPendant THEN
            TPReadFK respuesta,"Aceptar objetivos enviados por la PC?",stEmpty,stEmpty,stEmpty,"Iniciar","Cancelar"\MaxTime:=30\BreakFlag:=motivo;
            IF motivo <> 0 OR respuesta <> 4 THEN
                TPWrite "Servidor PC cancelado.";
                RETURN;
            ENDIF
        ENDIF
        vvPcFin := 0;
        vvRbCodigo := 0;
        vvRbSeq := vvPcSeq;
        vvRbActivo := 1;
        AccSet 50, 50;
        TPWrite "Servidor PC activo. Esperando objetivos.";
        WHILE vvPcFin = 0 DO
            IF vvPcSeq <> vvRbSeq THEN
                seq := vvPcSeq;
                objetivo := vvPcTarget;
                actual := CJointT();
                vvRbCodigo := Validar(objetivo.robax, actual.robax);
                IF vvRbCodigo = 0 THEN
                    vvRbCodigo := 1;
                    TPWrite "Moviendo orden"\Num:=seq;
                    objetivo.extax := actual.extax;
                    MoveAbsJ objetivo,vPc,fine,tool0;
                    vvRbCodigo := 2;
                ELSE
                    TPWrite "Orden rechazada, codigo"\Num:=vvRbCodigo;
                ENDIF
                vvRbSeq := seq;
            ENDIF
            WaitTime 0.1;
        ENDWHILE
        vvRbActivo := 0;
        AccSet 100, 100;
        TPWrite "Servidor PC terminado.";
    ENDPROC

    LOCAL PROC MostrarEjes(robjoint ejes)
        TPWrite "Objetivo absoluto en grados:";
        TPWrite "J1"\Num:=ejes.rax_1;
        TPWrite "J2"\Num:=ejes.rax_2;
        TPWrite "J3"\Num:=ejes.rax_3;
        TPWrite "J4"\Num:=ejes.rax_4;
        TPWrite "J5"\Num:=ejes.rax_5;
        TPWrite "J6"\Num:=ejes.rax_6;
    ENDPROC

    LOCAL FUNC bool ModoPermitido()
        VAR bool permitido := FALSE;
        IF OpMode() = OP_MAN_PROG THEN
            permitido := TRUE;
        ELSEIF permitirAuto THEN
            IF OpMode() = OP_AUTO THEN
                permitido := TRUE;
            ENDIF
        ENDIF
        RETURN permitido;
    ENDFUNC

    LOCAL FUNC num Validar(robjoint q, robjoint actual)
        IF ModoPermitido() = FALSE THEN
            RETURN -4;
        ENDIF
        IF Abs(q.rax_1) > 1000 OR Abs(q.rax_2) > 1000 OR Abs(q.rax_3) > 1000 OR
           Abs(q.rax_4) > 1000 OR Abs(q.rax_5) > 1000 OR Abs(q.rax_6) > 1000 THEN
            RETURN -1;
        ENDIF
        IF q.rax_1 < (-165 + margenDeg) OR q.rax_1 > (165 - margenDeg) OR
           q.rax_2 < (-110 + margenDeg) OR q.rax_2 > (110 - margenDeg) OR
           q.rax_3 < (-110 + margenDeg) OR q.rax_3 > (70 - margenDeg) OR
           q.rax_4 < (-160 + margenDeg) OR q.rax_4 > (160 - margenDeg) OR
           q.rax_5 < (-120 + margenDeg) OR q.rax_5 > (120 - margenDeg) OR
           q.rax_6 < (-400 + margenDeg) OR q.rax_6 > (400 - margenDeg) THEN
            RETURN -2;
        ENDIF
        IF Abs(q.rax_1 - actual.rax_1) > maxPasoDeg OR Abs(q.rax_2 - actual.rax_2) > maxPasoDeg OR
           Abs(q.rax_3 - actual.rax_3) > maxPasoDeg OR Abs(q.rax_4 - actual.rax_4) > maxPasoDeg OR
           Abs(q.rax_5 - actual.rax_5) > maxPasoDeg OR Abs(q.rax_6 - actual.rax_6) > maxPasoDeg THEN
            RETURN -3;
        ENDIF
        RETURN 0;
    ENDFUNC
ENDMODULE
