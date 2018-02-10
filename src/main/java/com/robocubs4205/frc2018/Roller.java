package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
public class Roller extends Subsystem{
    private final Spark motor = new Spark(7);

    private final double outPower = 1;
    private final double inPower = 1;

    {
        motor.setInverted(true);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Stop());
    }

    class Stop extends InstantCommand{
        Stop(){
            requires(Roller.this);
        }
        @Override
        protected void initialize(){
            motor.set(0);
        }
    }

    class Out extends PerpetualCommand{
        Out(){
            requires(Roller.this);
        }
        @Override
        protected void execute(){
            motor.set(outPower);
        }
    }

    class In extends PerpetualCommand{
        In(){
            requires(Roller.this);
        }
        @Override
        protected void execute(){
            motor.set(-inPower);
        }
    }
}
