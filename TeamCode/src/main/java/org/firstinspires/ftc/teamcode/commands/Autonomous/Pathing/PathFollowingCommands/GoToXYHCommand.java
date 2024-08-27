package org.firstinspires.ftc.teamcode.commands.Autonomous.Pathing.PathFollowingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;
import org.firstinspires.ftc.teamcode.utils.time.TDTTimer;

public class GoToXYHCommand extends CommandBase {
    private MecanumDriveTrain driveTrain;
    private final double x;
    private final double y;
    private final double heading;
    private final double movementSpeed;
    private final double stopTime;
    private final double startStopDistance;
    private final TDTTimer timer = new TDTTimer();

    public GoToXYHCommand(double x , double y , double heading , double movementSpeed , double stopTime , double startStopDistance){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.movementSpeed = movementSpeed;
        this.stopTime = stopTime;
        this.startStopDistance = startStopDistance;
    }

    @Override
    public void execute() {
        CurvePoint robotLocation = new CurvePoint(driveTrain.odometry.getX() , driveTrain.odometry.getY());

        double robot_movement_x = driveTrain.strafeController.getOutput(robotLocation.x , x);
        double robot_movement_y = driveTrain.forwardController.getOutput(robotLocation.y , y);
        double robot_movement_turn = driveTrain.turnController.getOutput(driveTrain.odometry.getHeading() , heading);

        driveTrain.fieldCentric(robot_movement_x , robot_movement_y , robot_movement_turn , movementSpeed);

        double displacement = Math.sqrt(Math.pow(y - robotLocation.y , 2) + Math.pow(x - robotLocation.x , 2));

        if(Math.abs(displacement) < startStopDistance){
            timer.beginStopState();
        }else{
            timer.beginStartTime();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.isUnderStopState(stopTime);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
