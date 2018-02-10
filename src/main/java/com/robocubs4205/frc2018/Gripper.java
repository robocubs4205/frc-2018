package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class Gripper extends Subsystem {
    private final double openSpeed = 0.75;
    private final double closeSpeed = 0.75;
    private final Talon rightMotor = new Talon(3);
    private final Talon leftMotor = new Talon(0);

    {
        leftMotor.setInverted(true);
    }

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

    class Open extends PerpetualCommand {

        Open() {
            requires(Gripper.this);
        }

        @Override
        protected void execute() {
            leftMotor.set(openSpeed);
            rightMotor.set(openSpeed);
        }
    }

    class Close extends PerpetualCommand {
        Close() {
            requires(Gripper.this);
        }

        @Override
        protected void execute() {
            leftMotor.set(-closeSpeed);
            rightMotor.set(-closeSpeed);
        }
    }
}
