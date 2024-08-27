package org.firstinspires.ftc.teamcode.commands.Autonomous.Pathing.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.time.TDTTimer;

public class TurnCommand extends CommandBase {
    private final double targetAngle;

    private final MecanumDriveTrain driveTrain;
    private final TDTTimer timer;

    public TurnCommand(double targetAngle , MecanumDriveTrain driveTrain){
        this.targetAngle = targetAngle;
        this.driveTrain = driveTrain;
        this.timer = new TDTTimer();
    }

    @Override
    public void execute() {
        double turnPower = 0; //driveTrain.turnController.getOutput(driveTrain.imu.getAngle() , targetAngle);

        driveTrain.tl.setPower(-turnPower);
        driveTrain.bl.setPower(-turnPower);
        driveTrain.tr.setPower(turnPower);
        driveTrain.br.setPower(turnPower);

        if(Math.abs(driveTrain.turnController.getError()) <= 2){
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
