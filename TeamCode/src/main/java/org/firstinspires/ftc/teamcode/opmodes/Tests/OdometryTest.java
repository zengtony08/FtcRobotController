package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control_systems.Odometry.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class OdometryTest extends CommandOpMode {
    private Robot robot;
    private StandardTrackingWheelLocalizer localizer;

    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();

    @Override
    public void initialize() {
        this.robot = new Robot(hardwareMap , telemetry);
        robot.driveTrain.setDtFloatMode();
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    }

    @Override
    public void run() {
        super.run();
        robot.driveTrain.odometry.updateTracker();
        localizer.update();
        telemetry.addData("x position : " , robot.driveTrain.odometry.getX());
        telemetry.addData("y position : " , robot.driveTrain.odometry.getY());
        telemetry.addData("heading : " , robot.driveTrain.odometry.getHeading());
        telemetry.addData("heading RAD: " , robot.driveTrain.odometry.getHeadingRAD());
        telemetry.addLine();
        telemetry.addData("RR Coords", localizer.getPose());
        telemetry.addData("RR Heading", localizer.getPose().getHeading());
        telemetry.update();
    }
}
