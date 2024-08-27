package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LynxCacheCommand extends CommandBase {
    private final List<LynxModule> lynxHubs;

    public LynxCacheCommand(HardwareMap hardwareMap){
        lynxHubs = hardwareMap.getAll(LynxModule.class);
    }

    @Override
    public void initialize() {
        for(LynxModule lynxHub : lynxHubs){
            lynxHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
