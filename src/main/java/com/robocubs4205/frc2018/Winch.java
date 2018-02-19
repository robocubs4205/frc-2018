package com.robocubs4205.frc2018;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class Winch extends Subsystem{
    private final double retractPower = 0.75;
    private final TalonSRX motor = new TalonSRX(14);

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Stop());
    }

    class Stop extends InstantCommand{
        Stop(){
            requires(Winch.this);
        }
        @Override
        protected void execute(){
            motor.set(ControlMode.PercentOutput,0);
        }
    }

    class Retract extends PerpetualCommand {
        Retract(){
            requires(Winch.this);
        }

        @Override
        protected void execute(){
            motor.set(ControlMode.PercentOutput, retractPower);
        }
    }
}
