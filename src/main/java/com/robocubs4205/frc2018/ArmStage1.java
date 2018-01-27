package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
public class ArmStage1 extends Subsystem{
    private final double extendSpeed = 1;
    private final double retractSpeed = 1;
    private final Talon motor = new Talon(1);
    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Stop());
    }

    class Stop extends InstantCommand{
        Stop(){
            requires(ArmStage1.this);
        }

        @Override
        protected void execute(){
            motor.set(0);
        }
    }

    class Extend extends Command {

        Extend(){
            requires(ArmStage1.this);
        }

        @Override
        protected void execute(){
            motor.set(-extendSpeed);
        }

        @Override
        protected boolean isFinished() {
            return false;
        }
    }

    class Retract extends Command {

        Retract(){
            requires(ArmStage1.this);
        }

        @Override
        protected void execute(){
            motor.set(retractSpeed);
        }

        @Override
        protected boolean isFinished() {
            return false;
        }
    }
}
