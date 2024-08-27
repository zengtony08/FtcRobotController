package org.firstinspires.ftc.teamcode.opmodes.Tests;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;

import java.util.ArrayList;
@TeleOp
public class followPathTest extends OpMode {
    private Robot robot;
    ArrayList<CurvePoint> path;
    @Override
    public void init() {
        robot = new Robot(hardwareMap , telemetry);
        path = new ArrayList<>();
        path.add(new CurvePoint(new Point(10, 10), 0.3, 45, 3));
        path.add(new CurvePoint(new Point(20, 20), 0.3, 90, 3));
        robot.driveTrain.followPath(path);
    }

    @Override
    public void loop() {

    }
}
