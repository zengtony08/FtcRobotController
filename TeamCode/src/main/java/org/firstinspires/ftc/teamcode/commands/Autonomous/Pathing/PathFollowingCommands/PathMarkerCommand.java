package org.firstinspires.ftc.teamcode.commands.Autonomous.Pathing.PathFollowingCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.control_systems.PointPursuit.PointPursuitPath;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;

import java.util.Map;

public class PathMarkerCommand extends CommandBase {
    private final PointPursuitPath path;
    private final MecanumDriveTrain driveTrain;
    private final Map<Double , CommandBase> commandMap;
    private final double pathLength;
    private double lastMarkerPassed = 0;

    public PathMarkerCommand(PointPursuitPath path , MecanumDriveTrain driveTrain ){
        this.path = path;
        this.commandMap = path.getMarkers();
        this.pathLength = path.getPathLength();
        this.driveTrain = driveTrain;
    }

    @Override
    public void execute() {
        CurvePoint robotLocation = new CurvePoint(driveTrain.odometry.getX() , driveTrain.odometry.getY());
        CurvePoint currIndexInPath = path.clipToPath(path , robotLocation.x , robotLocation.y);

        PointPursuitPath endPath = new PointPursuitPath();
        endPath.startingPoint(new CurvePoint(robotLocation.x , robotLocation.y));

        for(int i = currIndexInPath.pointIndex; i < path.size() - 1; i++){
            endPath.add(new CurvePoint(path.get(i)));
        }

        double progressPercent = (endPath.getPathLength() - pathLength) / pathLength;

        for(Map.Entry<Double, CommandBase> commandEntry : commandMap.entrySet()){
            double progress = commandEntry.getKey();
            Command command = commandEntry.getValue();

            if(progress > lastMarkerPassed && progress < progressPercent){
                CommandScheduler.getInstance().schedule(command);
                lastMarkerPassed = progress;
            }
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
