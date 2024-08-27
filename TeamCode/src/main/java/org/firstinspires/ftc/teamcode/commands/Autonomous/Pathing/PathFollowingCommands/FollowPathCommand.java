package org.firstinspires.ftc.teamcode.commands.Autonomous.Pathing.PathFollowingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.control_systems.PointPursuit.PointPursuitPath;
import org.firstinspires.ftc.teamcode.control_systems.PointPursuit.PursuitMath;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;
import org.firstinspires.ftc.teamcode.utils.Util;
import org.firstinspires.ftc.teamcode.utils.time.TDTTimer;

public class FollowPathCommand extends CommandBase {
    private final MecanumDriveTrain driveTrain;

    private final PursuitMath pursuitMath = new PursuitMath();

    private final PointPursuitPath path;
    private final PointPursuitPath headingPath;
    private final CurvePoint lastPoint;

    private final TDTTimer timer = new TDTTimer();

    public FollowPathCommand(MecanumDriveTrain driveTrain , PointPursuitPath path){
        this.driveTrain = driveTrain;
        this.path = path;
        this.lastPoint = path.get(path.size() - 1);
        this.headingPath  = new PointPursuitPath(path.getPath());
        headingPath.extendPath(50.0 + lastPoint.lookaheadDistance);
    }

    @Override
    public void execute() {
        CurvePoint robotLocation = new CurvePoint(driveTrain.odometry.getX() , driveTrain.odometry.getY());

        CurvePoint currPointInPath = path.clipToPath(path , robotLocation.x , robotLocation.y);
        CurvePoint currHeadingPointInPath = path.clipToPath(headingPath , robotLocation.x , robotLocation.y);


        CurvePoint lookaheadPoint = pursuitMath.lookAheadPoint(path, robotLocation , currPointInPath);


        CurvePoint headingPoint = pursuitMath.lookAheadPoint(headingPath , robotLocation ,
                currHeadingPointInPath);


        double actualRelativePointAngle = (Math.toRadians(path.get(currPointInPath.pointIndex).faceTowardsAngle) -Math.toRadians(90));
        double angleToPointRaw = Math.atan2(headingPoint.y-robotLocation.y,headingPoint.x-robotLocation.x) - Math.toRadians(90);

        double absolutePointAngle = angleToPointRaw+actualRelativePointAngle;

        double angleToPointWrapped = Util.angleWrap(absolutePointAngle);

        double robot_centric_x = driveTrain.strafeController.getOutput(robotLocation.x , lookaheadPoint.x);
        double robot_centric_y = driveTrain.forwardController.getOutput(robotLocation.y , lookaheadPoint.y);
        double robot_centric_turn = driveTrain.turnController.getOutput(driveTrain.odometry.getHeading() , Math.toDegrees(angleToPointWrapped));

        driveTrain.fieldCentric(robot_centric_x , robot_centric_y , robot_centric_turn , lookaheadPoint.movementSpeed);

        // GoToPosCommand(driveTrain , lookaheadPoint , headingPoint);


        double distanceFromEnd = Math.sqrt(Math.pow(lastPoint.y - driveTrain.odometry.getY() , 2) +
                Math.pow(lastPoint.x - driveTrain.odometry.getX() , 2));

        if(distanceFromEnd <= lastPoint.stopStartDistance){
            timer.beginStopState();
        }else{
            timer.beginStartTime();
        }

    }

    @Override
    public boolean isFinished() {
        return !timer.isUnderStopState(lastPoint.stopTime);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
