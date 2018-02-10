package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class Gripper extends Subsystem {
    private final double openSpeed = 0.75;
    private final double closeSpeed = 0.75;
    private final double outSpeed = 1;
    private final double inSpeed = 1;
    private final Talon rightMotor = new Talon(3);
    private final Talon leftMotor = new Talon(0);
    private final Spark rightBeltMotor = new Spark(5);
    private final Spark leftBeltMotor = new Spark(6);


    {
        leftMotor.setInverted(true);
        rightBeltMotor.setInverted(true);
        leftBeltMotor.setInverted(true);
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
            leftBeltMotor.set(0);
            rightBeltMotor.set(0);
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

    class Out extends PerpetualCommand {
        Out(){
            requires(Gripper.this);
        }

        @Override
        protected void execute(){
            leftBeltMotor.set(outSpeed);
            rightBeltMotor.set(outSpeed);
        }
    }

    class In extends PerpetualCommand {
        In(){
            requires(Gripper.this);
        }

        @Override
        protected void execute(){
            leftBeltMotor.set(-inSpeed);
            rightBeltMotor.set(-inSpeed);
        }
    }
}
