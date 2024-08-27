package org.firstinspires.ftc.teamcode.control_systems.PIDController;

public interface PIDConstants {
    double DRIVE_KP = 0;
    double DRIVE_KI = 0.0;
    double DRIVE_KD = 0.0;

    double TURN_KP = 0;
    double TURN_KI = 0.0;
    double TURN_KD = 0;

    double STRAFE_KP = 0.0;
    double STRAFE_KI = 0.0;
    double STRAFE_KD = 0.0;

    double ARM_KP = 0.0;
    double ARM_KI = 0.0;
    double ARM_KD = 0.0;

    double PURE_DRIVE_KP = 0.04; // 0.28
    double PURE_STRAFE_KP = 0.04; //0.25
    double PURE_TURN_KP = 0.00975;

    double PURE_DRIVE_KI = 0;
    double PURE_STRAFE_KI = 0;
    double PURE_TURN_KI = 0;

    double PURE_DRIVE_KD = 2.5; //2,8
    double PURE_STRAFE_KD = 1.9; //2.25
    double PURE_TURN_KD = 0;
}
