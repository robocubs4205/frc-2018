package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
public class ArmStage2 extends Subsystem {
    private double proportionalSpeed = 1;
    private final Talon motor = new Talon(2);

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Stop());
    }

    class Stop extends InstantCommand {
        Stop() {
            requires(ArmStage2.this);
        }

        @Override
        protected void execute() {
            motor.set(0);
        }
    }

    class Proportional extends PerpetualCommand {
        private final double value;

        Proportional(double value) {
            this.value = value;
            requires(ArmStage2.this);
        }

        @Override
        protected void execute() {
            motor.set(value * proportionalSpeed);
        }
    }
}
