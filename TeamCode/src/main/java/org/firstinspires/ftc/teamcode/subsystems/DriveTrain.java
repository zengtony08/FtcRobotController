package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control_systems.PIDController.PIDController;
import org.firstinspires.ftc.teamcode.control_systems.PIDController.PIDTriplet;
import org.firstinspires.ftc.teamcode.utils.robot_hardware.Motor;


public class DriveTrain extends SubsystemBase {
    public Motor tl;
    public Motor bl;
    public Motor tr;
    public Motor br;
    public PIDController forwardController;
    public PIDController turnController;
    public PIDController strafeController;

    public boolean testingMode;

    public DriveTrain(String tlName , String blName , String trName , String brName , PIDTriplet DrivePID , PIDTriplet StrafePID , PIDTriplet TurnPID, HardwareMap hardwareMap){
        tl = new Motor(tlName , hardwareMap);
        bl = new Motor(blName ,  hardwareMap);
        tr = new Motor(trName , hardwareMap);
        br = new Motor(brName , hardwareMap);

        tl.reverse();
        bl.reverse();

        turnController = new PIDController(TurnPID.getKP() , TurnPID.getKI() , TurnPID.getKD());
        forwardController = new PIDController(DrivePID.getKP() , DrivePID.getKI() , DrivePID.getKD());
        strafeController = new PIDController(StrafePID.getKP() , StrafePID.getKI() , StrafePID.getKD());

        setDtBreakMode();
/*
        resetAllMotors();
        setDtBreakMode();

        driveController = new PIDController(DrivePID.getKP() , DrivePID.getKI() , DrivePID.getKD());
        turnController = new PIDController(TurnPID.getKP() , TurnPID.getKI() , TurnPID.getKD());
        strafeController = new PIDController(StrafePID.getKP() , StrafePID.getKI() , StrafePID.getKD());*/
    }

    public void stop(){
        tl.setPower(0);
        bl.setPower(0);
        tr.setPower(0);
        br.setPower(0);
    }

    public void setMotorZeroBehaviors(DcMotor.ZeroPowerBehavior zeroBehaviors){
        if(zeroBehaviors.equals(DcMotor.ZeroPowerBehavior.BRAKE)) {
            bl.setBreakMode();
            br.setBreakMode();
            tl.setBreakMode();
            bl.setBreakMode();
        }else if (zeroBehaviors.equals(DcMotor.ZeroPowerBehavior.FLOAT)){
            bl.setFloatMode();
            br.setFloatMode();
            tl.setFloatMode();
            tr.setFloatMode();
        }
    }

    public void setDriveControllerConstants(double kp , double ki , double kd){

        forwardController.setConstants(kp , ki , kd);
    }

    public void setTurnControllerContstants(double kp , double ki , double kd){
        turnController.setConstants(kp , ki , kd);
    }

    public void resetAllMotors(){
        if(tl != null && bl != null && tr != null && br != null){
            tl.resetMotor();
            bl.resetMotor();
            tr.resetMotor();
            br.resetMotor();
        }
    }

    public void reverseRightSide(){
        tr.reverse();
        br.reverse();
    }

    public void reverseLeftSide(){
        tl.reverse();
        bl.reverse();
    }

    public void setDtBreakMode(){
        setMotorZeroBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDtFloatMode(){
        setMotorZeroBehaviors(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setTestingMode(boolean testingMode){
        this.testingMode = testingMode;
    }


}
