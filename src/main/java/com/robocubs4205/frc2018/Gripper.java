package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class Gripper extends Subsystem {
    private final double openSpeed = 0.5;
    private final double closeSpeed = 0.5;
    private final Talon rightMotor = new Talon(3);
    private final Talon leftMotor = new Talon(0);


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Stop());
    }

    class Stop extends InstantCommand {
        Stop() {
            requires(Gripper.this);
        }

        @Override
        protected void execute() {
            rightMotor.set(0);
            leftMotor.set(0);
        }
    }

    class Open extends Command {

        Open() {
            requires(Gripper.this);
        }

        @Override
        protected void execute() {
            leftMotor.set(openSpeed);
            rightMotor.set(-openSpeed);
        }

        @Override
        protected boolean isFinished() {
            return false;
        }
    }

    class Close extends Command {
        Close() {
            requires(Gripper.this);
        }

        @Override
        protected void execute() {
            leftMotor.set(-closeSpeed);
            rightMotor.set(closeSpeed);
        }

        @Override
        protected boolean isFinished() {
            return false;
        }
    }
}
