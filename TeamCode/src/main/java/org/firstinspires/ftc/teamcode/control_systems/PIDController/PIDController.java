package org.firstinspires.ftc.teamcode.control_systems.PIDController;

import com.qualcomm.robotcore.util.Range;

public class PIDController{
    private double KP;
    private double KI;
    private double KD;
    private double previousTime = 0;
    private double error;
    private double i = 0;
    private double d = 0;
    private double deltaTime = 0;
    private double previousError = 0;

    public PIDController(double KP , double KI , double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    public double getOutput(double currPos , double targetPos){
        double currTime = System.currentTimeMillis();

        error = targetPos - currPos;
        double P = KP * error;
        deltaTime = currTime - previousTime;
        i += currPos > targetPos * 0.8 ? deltaTime * error : 0;
        double I = KI * i;
        d = (error - previousError) / deltaTime;
        double D = KD * d;

        previousTime = currTime;
        previousError = error;

        return Range.clip(P + I + D , -1 , 1);
    }

    public double getOutputF(double currentRPM , double targetRPM , double MAX_RPM){
        error = targetRPM - currentRPM;
        double P = KP * error;
        deltaTime = System.currentTimeMillis() - previousTime;
        i += currentRPM > targetRPM * 0.8 ? deltaTime * error : 0;
        double I = KI * i;
        d = (error - previousError) / deltaTime;
        double D = KD * d;

        previousTime = System.currentTimeMillis();
        previousError = error;

        double F = targetRPM / MAX_RPM;

        return Range.clip(P + I + D + F, -1 , 1);
    }
    
    public void setConstants(double KP , double KI , double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    public double getError(){
        return this.error;
    }


}
