package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.control_systems.Odometry.ThreeWheelTracker;

public class UpdateOdometryCommand extends CommandBase {
    private final ThreeWheelTracker odometry;

    public UpdateOdometryCommand(ThreeWheelTracker odometry){
        this.odometry = odometry;
    }

    @Override
    public void execute() {
        this.odometry.updateTracker();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
