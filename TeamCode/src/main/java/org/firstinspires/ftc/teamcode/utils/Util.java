package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

public class Util {
    public static double angleWrap(double angle){
        while(angle >= Math.PI){
            angle -= (Math.PI * 2.0);
        }
        while(angle <= -Math.PI){
            angle += (Math.PI * 2.0);
        }

        return angle;
    }

    public static double angleWrapDegrees(double angle) {
        while(angle > 180) {
            angle -= 360;
        }
        while (angle <= -180){
            angle += 360;
        }

        return angle;
    }

    public static void setInRange(double[] powerArray, double min, double max){
        for(int i = 0; i < powerArray.length; i ++){
            powerArray[i] = Range.clip(powerArray[i], min , max);
        }
    }

    public static void normalize(double[] powers){
        double max = 1;
        for(int i = 0 ; i < powers.length ; i ++){
            if(Math.abs(powers[i]) > max){
                max = Math.abs(powers[i]);
            }
        }

        if(max > 1){
            for(int i = 0 ; i < powers.length ; i ++){
                powers[i] /= max;
            }
        }
    }

    public static String pose2dToString(Pose2d pose2d) {
        return String.format("(%.3f, %.3f, %.3f°)", pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

}
