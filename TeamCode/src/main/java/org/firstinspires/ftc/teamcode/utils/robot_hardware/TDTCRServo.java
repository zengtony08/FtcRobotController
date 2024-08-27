package org.firstinspires.ftc.teamcode.utils.robot_hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TDTCRServo {
    private final CRServo servo;

    public TDTCRServo(String name , HardwareMap hardwareMap){
        servo = hardwareMap.get(CRServo.class , name);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        servo.setDirection(direction);
    }

    public void setPower(double power){
        servo.setPower(power);
    }

    public void getPower(double power){
        servo.getPower();
    }

}
