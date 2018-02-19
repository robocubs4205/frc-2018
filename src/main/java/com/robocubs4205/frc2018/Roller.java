package com.robocubs4205.frc2018;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class Roller extends Subsystem{
    private final WPI_TalonSRX motor = new WPI_TalonSRX(15);

    private final double outPower = 1;
    private final double inPower = 0.25;

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
