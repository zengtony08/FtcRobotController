package org.firstinspires.ftc.teamcode.control_systems.PIDController;

public class PIDTriplet {
    private double KP;
    private double KI;
    private double KD;

    public PIDTriplet(double KP , double KI , double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    public void setKP(double KP) {
        this.KP = KP;
    }

    public void setKI(double KI) {
        this.KI = KI;
    }

    public void setKD(double KD) {
        this.KD = KD;
    }

    public double getKP() {
        return KP;
    }

    public double getKI() {
        return KI;
    }

    public double getKD() {
        return KD;
    }
}
