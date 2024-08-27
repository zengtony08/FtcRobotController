package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control_systems.PIDController.PIDConstants;
import org.firstinspires.ftc.teamcode.control_systems.PIDController.PIDTriplet;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.robot_hardware.Motor;

import java.util.ArrayList;

public class Robot implements ROBOT_CONSTANTS, PIDConstants {
    public MecanumDriveTrain driveTrain;

    private final ArrayList<Motor> motorsUsingEncoders;


    public Robot(HardwareMap hwmap , Telemetry telemetry){
        driveTrain = new MecanumDriveTrain(tlName , blName , trName , brName , new PIDTriplet(DRIVE_KP , DRIVE_KI , DRIVE_KD) ,
                new PIDTriplet(STRAFE_KP , STRAFE_KI , STRAFE_KD) , new PIDTriplet(TURN_KP , TURN_KI , TURN_KD) , trackWidth , auxWidth , hwmap , telemetry);
       /* spinner = new Spinner(spinnerName , hwmap);
        arm = new Arm( armLName , armRName , new PIDTriplet(ARM_KP , ARM_KI , ARM_KD) , hwmap);
        intake = new Intake(intakeLName , intakeRName , hwmap);
*/
        driveTrain.setDtBreakMode();
        motorsUsingEncoders = new ArrayList<>();
        motorsUsingEncoders.add(driveTrain.odometry.LEncoder);
        motorsUsingEncoders.add(driveTrain.odometry.REncoder);
        motorsUsingEncoders.add(driveTrain.odometry.HEncoder);
    }


    public void resetAllMotors(){
        driveTrain.resetDtMotors();
    }

    public ArrayList<Motor> getMotorsUsingEncoders(){
        return this.motorsUsingEncoders;
    }


}