package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.time.Instant;

@SuppressWarnings("FieldCanBeLocal")
class ArmTilt extends Subsystem{
    private final double raisePower = 1;
    private final double lowerPower = 1;
    private final Talon master = new Talon(4);
    private final Talon slave = new Talon(7);

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
            master.set(0);
            slave.set(master.getSpeed());
        }
    }

    class Raise extends PerpetualCommand {

        Raise(){
            requires(ArmTilt.this);
        }

        @Override
        protected void execute(){
            master.set(raisePower);
            slave.set(master.getSpeed());
        }
    }

    class Lower extends PerpetualCommand {
        Lower(){
            requires(ArmTilt.this);
        }

        @Override
        protected void execute(){
            master.set(-lowerPower);
            slave.set(master.getSpeed());
        }
    }
}
