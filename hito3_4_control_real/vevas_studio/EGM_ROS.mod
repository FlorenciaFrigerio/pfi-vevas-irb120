MODULE EGM_ROS
    !==================================================================================
    ! EGM joint-position module for ROS 2 (abb_ros2 / abb_hardware_interface)
    ! Robot: IRB 120-3/0.6 - IRC5 Compact - RobotWare 6.12.01
    !
    ! Adapted from abb_ros2/robot_studio_resources/TRob1Main.mod
    ! (Copyright (c) 2018, ABB Schweiz AG - BSD-3-Clause)
    !
    ! Differences from the original:
    !   * NO "PROC main": the existing Module1 (production program) keeps its main.
    !     Start this with FlexPendant -> Debug -> "PP to Routine..." -> ros_egm
    !   * NO automatic MoveAbsJ to home: the robot does not move on start.
    !     Jog it to a safe position by hand before starting.
    !   * MaxSpeedDeviation lowered from 20 to 10 deg/s for the first tests.
    !     ROS side (mover_pose_ik.py, saludo.py) keeps joint speed <= 8 deg/s (80 %).
    !
    ! Version history (IMT-342, see docs/bitacora.md):
    !   v1 (2026-11-12) CondTime:=5  RampOutTime:=5  -> loaded on the controller 12-11.
    !        Problem (B-017): after 5 s converged EGMRunJoint ends, ramps out for 5 s and the
    !        WHILE loop calls it again; trajectories sent in that window are followed up to ~5 s late.
    !   v2 (2026-11-16) CondTime:=1000000 (EGMRunJoint never ends by convergence while ROS streams)
    !                   RampOutTime:=1     (shorter ramp-out when it does stop)
    !        NOT loaded yet: needs authorization (writes to the controller). Not syntax-checked on a
    !        controller or in RobotStudio; only numeric arguments changed vs v1.
    !
    ! Requires: UDPUC device "ROB_1" -> Remote Address 192.168.125.10, port 6511
    !           (Configuration -> Communication -> Transmission Protocol)
    !==================================================================================

    ! EGM identifier.
    LOCAL VAR egmident egm_id;

    ! Convergence condition [deg] (same as the original).
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    ! Maximum speed deviation allowed from EGM [deg/s].
    ! Raise it (original: 20) only once everything works.
    LOCAL CONST num max_speed_dev := 10;

    ! EGMRunJoint end condition: time [s] the position must stay within egm_condition before
    ! EGMRunJoint finishes. Very large -> it does not finish while ROS keeps streaming (v2).
    LOCAL CONST num cond_time := 1000000;

    ! Ramp-out time [s] when EGMRunJoint finishes or is stopped (v1: 5).
    LOCAL CONST num ramp_out_time := 1;

    PROC ros_egm()
        TPWrite "EGM_ROS: waiting for ROS 2 (UDP ROB_1 -> 192.168.125.10:6511)";

        ! Register an EGM id.
        EGMGetId egm_id;

        ! Joint-mode EGM over the UDPUC device ROB_1, with the "default" MOC profile.
        EGMSetupUC ROB_1, egm_id, "default", "ROB_1", \Joint;

        ! Prepare the EGM session.
        EGMActJoint egm_id
                    \J1:=egm_condition
                    \J2:=egm_condition
                    \J3:=egm_condition
                    \J4:=egm_condition
                    \J5:=egm_condition
                    \J6:=egm_condition
                    \MaxSpeedDeviation:=max_speed_dev;

        WHILE TRUE DO
            ! Run EGM: the robot follows the positions sent by ROS 2.
            EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=cond_time \RampOutTime:=ramp_out_time;
        ENDWHILE

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            ! ROS 2 is not sending data (timeout): keep waiting.
            TPWrite "EGM_ROS: communication timeout, retrying";
            TRYNEXT;
        ENDIF
    ENDPROC

    ! Run this (PP to Routine -> ros_egm_reset) if a previous session was
    ! aborted and EGMGetId/EGMSetupUC complains that the id is in use.
    PROC ros_egm_reset()
        EGMReset egm_id;
        TPWrite "EGM_ROS: egm_id released";
    ENDPROC
ENDMODULE
