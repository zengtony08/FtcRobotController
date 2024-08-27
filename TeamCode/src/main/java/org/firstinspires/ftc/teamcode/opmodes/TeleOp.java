package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    private Robot robot;
    private GamepadEx D1;
    private GamepadEx D2;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap , telemetry);
        robot.resetAllMotors();
        D1 = new GamepadEx(gamepad1);
        D2 = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        super.run();

        robot.driveTrain.robotCentric(D1.getLeftX() , D1.getLeftY() , -D1.getRightX());

    }
}
