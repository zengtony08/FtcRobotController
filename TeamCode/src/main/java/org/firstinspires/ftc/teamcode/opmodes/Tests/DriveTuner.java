package org.firstinspires.ftc.teamcode.opmodes.Tests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;
import org.firstinspires.ftc.teamcode.utils.Util;

@TeleOp
@Config
public class DriveTuner extends OpMode {
    private Logger _logger;
    private Robot robot;
    private FtcDashboard dashboard;

    // Declare the static PID Contstants.
    public static double KP = 0;
    public static double KI = 0;
    public static double KD = 0;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetH = 0;

    @Override
    public void init() {
        _logger = new Logger("DriveTuner", 3);
        dashboard = FtcDashboard.getInstance();
        // This allows us to output telemetry to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry , dashboard.getTelemetry());

        /* We can pass in the multipleTelemtry instance now and
        the tuner method will use it to output telemetry */
        robot = new Robot(hardwareMap , telemetry);
    }

    @Override
    public void loop() {
//        robot.driveTrain.tempTurn.setPID(KP , KI , KD);
        // Since we only want to move forward,
//        set the y target position to 20 and everything else to 0.
//        This will move the robot in the forward direction 20 inches.*/
        robot.driveTrain.goToPosition(new CurvePoint(targetX, targetY) , new CurvePoint(targetH));
        telemetry.addData("odometry X : ", robot.driveTrain.localizer.getPose().getX());
        telemetry.addData("Odometry Y : ", robot.driveTrain.localizer.getPose().getY());
        telemetry.addData("Odometry Heading : ", Math.toDegrees(robot.driveTrain.localizer.getPose().getHeading()));
        telemetry.update();
        _logger.info("Info Position : %s",
                Util.pose2dToString(robot.driveTrain.localizer.getPose()));
//        _logger.debug("Debug Position%s : ", robot.driveTrain.localizer.getPose());
    }

}