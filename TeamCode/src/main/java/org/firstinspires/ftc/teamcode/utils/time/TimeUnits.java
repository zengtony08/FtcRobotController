package org.firstinspires.ftc.teamcode.utils.time;

public enum TimeUnits{
    NANOSECS(1),
    MILLISECS(1E6),
    SECS(1E9),
    MINS(6e+10);
    public final double value;
    TimeUnits (double value){this.value = value;}
}