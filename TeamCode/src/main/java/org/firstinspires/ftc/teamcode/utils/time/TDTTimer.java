package org.firstinspires.ftc.teamcode.utils.time;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TDTTimer {


    private long startTime = 0L;
    private boolean isPaused = false;
    private long pauseStart = 0L;
    private long pauseLength = 0L;
    private double stopState = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public TDTTimer(){this.reset();}

    public void reset(){
        isPaused = false;
        pauseStart = 0L;
        pauseLength = 0L;
        startTime = 0L;
        stopState = 0;
    }

    public long nanoSeconds(){ return System.nanoTime(); }

    public long milliSeconds(){return System.currentTimeMillis();}

    public double seconds(){return System.nanoTime() / TimeUnits.SECS.value; }

    public void beginStartTime(){
        startTime = milliSeconds();
    }

    public double getStartTime(){return startTime;}

    public void beginStopState(){
        stopState = milliSeconds() - startTime;
    }

    public double getStopState(){return stopState;}

    public boolean isUnderStopState(double time){
        return getStopState() <= time;
    }

    public boolean elapsedTime(double time, TimeUnits type){
        return System.currentTimeMillis() - getStartTime() > (long)(time*type.value);
    }

    public boolean timeOut(double time){
        return System.currentTimeMillis() - getStartTime() > time;
    }
/*
    public long beginStartTime(){
        beginStartTime = System.nanoTime();
        //this.beginStartTime = beginStartTime;
        return beginStartTime;
    }*/

    public void pause(){
        if (!isPaused) pauseStart = System.nanoTime();
        isPaused = true;
    }

    public void resume(){
        if (isPaused) pauseLength += (System.nanoTime() - pauseStart);
        isPaused = false;
    }

    public boolean isPaused(){return isPaused;}
}
