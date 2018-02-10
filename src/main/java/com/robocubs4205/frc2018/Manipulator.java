package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class Manipulator {
    final Gripper gripper = new Gripper();
    final Belt belt = new Belt();

    class Stop extends CommandGroup{
        Stop(){
            addParallel(gripper.new Stop());
            addParallel(belt.new Stop());
        }
    }

    class Gripper extends Subsystem{
        private final Talon rightMotor = new Talon(3);
        private final Talon leftMotor = new Talon(0);

        private final double openPower = 0.75;
        private final double closePower = 0.75;

        private Gripper(){
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
                leftMotor.set(openPower);
                rightMotor.set(openPower);
            }
        }

        class Close extends PerpetualCommand {
            Close() {
                requires(Gripper.this);
            }

            @Override
            protected void execute() {
                leftMotor.set(-closePower);
                rightMotor.set(-closePower);
            }
        }
    }

    class Belt extends Subsystem{
        private final Spark rightMotor = new Spark(5);
        private final Spark leftMotor = new Spark(6);

        private final double outPower = 1;
        private final double inPower = 1;

        private Belt(){
            rightMotor.setInverted(true);
            leftMotor.setInverted(true);
        }

        @Override
        protected void initDefaultCommand() {
            setDefaultCommand(new Stop());
        }

        class Stop extends InstantCommand {
            Stop() {
                requires(Belt.this);
            }

            @Override
            protected void execute() {
                leftMotor.set(0);
                rightMotor.set(0);
            }
        }

        class Out extends PerpetualCommand {
            Out(){
                requires(Belt.this);
            }

            @Override
            protected void execute(){
                leftMotor.set(outPower);
                rightMotor.set(outPower);
            }
        }

        class In extends PerpetualCommand {
            In(){
                requires(Belt.this);
            }

            @Override
            protected void execute(){
                leftMotor.set(-inPower);
                rightMotor.set(-inPower);
            }
        }
    }
}
