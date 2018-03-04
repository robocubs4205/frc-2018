package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class ArmStage1 extends Subsystem{
    private final double extendPower = 1;
    private final double retractPower = 0.5;
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

    class Extend extends PerpetualCommand {

        private final double power;

        Extend(){
            this(extendPower);
        }

        Extend(double power){
            this.power = power;
            requires(ArmStage1.this);
        }

        @Override
        protected void execute(){
            motor.set(Math.abs(power));
        }
    }

    class Retract extends PerpetualCommand {

        Retract(){
            requires(ArmStage1.this);
        }

        @Override
        protected void execute(){
            motor.set(-retractPower);
        }
    }

    class Hold extends PerpetualCommand  {
        Hold(){
            requires(ArmStage1.this);
        }
        @Override
        protected void execute(){
            motor.set(0.3);
        }
    }
}
