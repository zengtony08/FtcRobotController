package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
@TeleOp
public class EncodersReverseTest extends CommandOpMode {
    private Robot robot;

    @Override
    public void initialize() {
        this.robot = new Robot(hardwareMap , telemetry);
    }


    @Override
    public void run() {
        super.run();
        robot.driveTrain.odometry.updateTracker();
        telemetry.addData("left inches : " , robot.driveTrain.odometry.getLeftInches());
        telemetry.addData("Right inches : " , robot.driveTrain.odometry.getRightInches());
        telemetry.addData("Horizontal inches : " , robot.driveTrain.odometry.getHorizontalInches());
        telemetry.update();
    }
}
