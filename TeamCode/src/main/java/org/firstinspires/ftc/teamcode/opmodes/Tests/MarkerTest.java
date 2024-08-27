package org.firstinspires.ftc.teamcode.opmodes.Tests;


import android.graphics.Point;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Autonomous.Pathing.PathFollowingCommands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.Autonomous.Pathing.PathFollowingCommands.PathMarkerCommand;
import org.firstinspires.ftc.teamcode.commands.LynxCacheCommand;
import org.firstinspires.ftc.teamcode.commands.UpdateOdometryCommand;
import org.firstinspires.ftc.teamcode.control_systems.PointPursuit.PointPursuitPath;
import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;

public class MarkerTest extends CommandOpMode {
    private Robot robot;
    private PointPursuitPath intakePath;

    @Override
    public void initialize() {
        this.robot = new Robot(hardwareMap , telemetry);
        intakePath = new PointPursuitPath();
        intakePath.add(new CurvePoint(new Point(0 , 0), 0.75 , 90 , 0 , 0));
        intakePath.add(new CurvePoint(new Point(0 , 20), 0.75 , 90 , 0 , 0));
        intakePath.add(new CurvePoint(new Point(20 , 20), 0.75 , 90 , 500 , 2));

    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        CommandScheduler.getInstance().schedule(
                new LynxCacheCommand(hardwareMap)
                .andThen(new UpdateOdometryCommand(robot.driveTrain.odometry))
        );

        CommandScheduler.getInstance().schedule(
                new PathMarkerCommand(intakePath , robot.driveTrain)
                .deadlineWith(new FollowPathCommand(robot.driveTrain , intakePath))
        );

    }



}
