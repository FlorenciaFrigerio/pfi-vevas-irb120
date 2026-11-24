MODULE VEVAS_RWS
  ! VEVAS (de LabJointRws V3). vvRbCodigo: 0 listo,1 mueve,2 llego,-1 vacio,-2 lim,-3 paso,-4 modo
  ! C02: MoveAbsJ \T con tiempo >= kPico*|dq|/vLim por eje (vLim = 5 % de la ficha). <4800 car.
  LOCAL CONST num maxPasoDeg := 45;
  LOCAL CONST num margenDeg := 2;
  LOCAL CONST bool permitirAuto := TRUE;
  LOCAL CONST bool confirmarEnPendant := TRUE;
  LOCAL CONST speeddata vPc := [20,5,20,5];
  LOCAL CONST num vLim{6} := [12.5,12.5,12.5,16,16,21];
  LOCAL CONST num kPico := 2.5;
  PERS jointtarget vvPcTarget := [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]];
  PERS num vvPcSeq := 0;
  PERS num vvPcFin := 0;
  PERS num vvRbSeq := 0;
  PERS num vvRbCodigo := 0;
  PERS num vvRbActivo := 0;
  PERS num vvRbTiempo := 0;

  PROC VevasServidor()
    VAR jointtarget objetivo;
    VAR jointtarget actual;
    VAR num seq := 0;
    VAR num respuesta := 0;
    VAR errnum motivo := 0;
    VAR num t := 0;

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
          t := Tiempo(objetivo.robax, actual.robax);
          vvRbTiempo := t;
          MoveAbsJ objetivo,vPc\T:=t,fine,tool0;
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

  LOCAL FUNC num Tj(num t, num d, num v)
    IF kPico * Abs(d) / v > t THEN
      RETURN kPico * Abs(d) / v;
    ENDIF
    RETURN t;
  ENDFUNC

  LOCAL FUNC num Tiempo(robjoint q, robjoint a)
    VAR num t := 0.5;
    t := Tj(t, q.rax_1 - a.rax_1, vLim{1});
    t := Tj(t, q.rax_2 - a.rax_2, vLim{2});
    t := Tj(t, q.rax_3 - a.rax_3, vLim{3});
    t := Tj(t, q.rax_4 - a.rax_4, vLim{4});
    t := Tj(t, q.rax_5 - a.rax_5, vLim{5});
    t := Tj(t, q.rax_6 - a.rax_6, vLim{6});
    RETURN t;
  ENDFUNC

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
