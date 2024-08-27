package org.firstinspires.ftc.teamcode.commands.Autonomous.Pathing.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.time.TDTTimer;

import java.util.ArrayList;
import java.util.List;

public class DriveForward extends CommandBase {
    private final double distance;
    private double initialAngle;
    private final double movementSpeed;
    private final boolean firstTimeRunning = true;

    private final MecanumDriveTrain driveTrain;
    private final TDTTimer timer;

    public DriveForward(double distance , double movementSpeed , MecanumDriveTrain driveTrain){
        this.distance = distance;
        this.movementSpeed = movementSpeed;
        this.driveTrain = driveTrain;
        this.timer = new TDTTimer();
    }

    @Override
    public void execute() {
        // if(firstTimeRunning){
        // initialAngle = driveTrain.imu.getAngle();
        // driveTrain.resetDtMotors();
        //}

        double drive = driveTrain.forwardController.getOutput(driveTrain.tl.getCurrPosInches() , distance);
        double turn = 0; //driveTrain.turnController.getOutput(driveTrain.imu.getAngle() , initialAngle);

        double leftPower = drive - turn;
        double rightPower = drive + turn;

        List<Double> powerList = new ArrayList<>();
        powerList.add(leftPower);
        powerList.add(rightPower);

        driveTrain.normalizeMecanumPowers(powerList);

        driveTrain.tl.setPower(leftPower);
        driveTrain.bl.setPower(leftPower);
        driveTrain.tr.setPower(rightPower);
        driveTrain.br.setPower(rightPower);

        if(Math.abs(driveTrain.forwardController.getError()) <= 2){
            timer.beginStopState();
        }else{
            timer.beginStartTime();
        }

    }

    @Override
    public boolean isFinished() {
        return timer.isUnderStopState(500);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
