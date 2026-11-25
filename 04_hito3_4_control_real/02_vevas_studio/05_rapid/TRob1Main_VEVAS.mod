MODULE TRob1Main
!======================================================================================================
! Basado en TRob1Main.mod de PickNikRobotics/abb_ros2 (robot_studio_resources), que a su vez viene del
! ejemplo de abb_libegm. Copyright (c) 2018, ABB Schweiz AG. Licencia BSD-3 (ver el archivo original).
!
! Cambios del Grupo VEVAS (IRB 120, IMT-342):
!   1. NO va a "home" al arrancar. El original hace MoveAbsJ a [0,0,0,0,30,0] a v200: al pulsar PLAY el
!      robot se movería solo hasta esa postura. Aquí se usa la postura ACTUAL (CJointT) para crear el
!      punto fino que EGM necesita, así que al arrancar el robot NO se mueve.
!   2. Mensajes en el FlexPendant para saber en qué estado está.
!   3. Mismo protocolo de comunicación "ROB_1" (UDPUC) que espera el driver en el puerto 6511.
!
! Requisitos en el controlador (los configura el encargado del laboratorio):
!   Controller -> Configuration -> Communication -> Transmission Protocol -> ROB_1
!       Type: UDPUC   Remote Address: 192.168.125.10 (IP del Ubuntu)   Remote port: 6511   Local port: 0
!   Este módulo cargado en la tarea T_ROB1 (si T_ROB1 ya tiene otro "main", NO borrar nada: consultar).
!======================================================================================================

    ! Identificador del proceso EGM
    LOCAL VAR egmident egm_id;
    ! Convergencia: EGM considera que llegó cuando cada junta está a menos de 0.1 grados de la referencia
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];
    ! Postura actual (para el punto fino inicial sin movimiento)
    LOCAL VAR jointtarget actual;

    PROC main()
        TPErase;
        TPWrite "VEVAS: inicio EGM. El robot NO se mueve al arrancar.";
        actual := CJointT();
        MoveAbsJ actual, v10, fine, tool0;

        EGMGetId egm_id;
        EGMSetupUC ROB_1, egm_id, "default", "ROB_1", \Joint;
        EGMActJoint egm_id
                    \J1:=egm_condition
                    \J2:=egm_condition
                    \J3:=egm_condition
                    \J4:=egm_condition
                    \J5:=egm_condition
                    \J6:=egm_condition
                    \MaxSpeedDeviation:=20.0;
        TPWrite "VEVAS: EGM listo, esperando datos de ROS 2 (puerto 6511)...";
        WHILE TRUE DO
            ! Sigue las posiciones que manda ROS. Si ROS deja de mandar, el robot se detiene (STOP_HOLD).
            EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=5 \RampOutTime:=5;
        ENDWHILE
        EGMReset egm_id;

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "VEVAS: sin datos de ROS 2 (timeout UDP). Reintentando...";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
