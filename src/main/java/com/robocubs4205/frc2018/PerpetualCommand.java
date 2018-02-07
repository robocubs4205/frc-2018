package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.command.Command;

abstract class PerpetualCommand extends Command{
    @Override
    protected boolean isFinished() {
        return false;
    }
}
