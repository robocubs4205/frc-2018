package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class ArmTilt extends Subsystem{
    private final double raisePower = 1;
    private final double lowerPower = 1;
    private final Talon motor = new Talon(4);

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Stop());
    }

    class Stop extends InstantCommand{
        Stop(){
            requires(ArmTilt.this);
        }
        @Override
        protected void execute(){
            motor.set(0);
        }
    }

    class Raise extends PerpetualCommand {

        Raise(){
            requires(ArmTilt.this);
        }

        @Override
        protected void execute(){
            motor.set(raisePower);
        }
    }

    class Lower extends PerpetualCommand {
        Lower(){
            requires(ArmTilt.this);
        }

        @Override
        protected void execute(){
            motor.set(-lowerPower);
        }
    }
}
