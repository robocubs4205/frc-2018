package com.robocubs4205.frc2018;

import com.ctre.phoenix.drive.DriveMode;
import com.ctre.phoenix.drive.MecanumDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
class DriveTrain extends Subsystem {
    private final MecanumDrive drive;

    private final double proportionalLateralSpeed = 1;
    private final double proportionalTurnSpeed = 1;

    DriveTrain() {
        TalonSRX frontLeft = new TalonSRX(13);
        TalonSRX frontRight = new TalonSRX(11);
        TalonSRX rearLeft = new TalonSRX(12);
        TalonSRX rearRight = new TalonSRX(10);

        drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Stop());
    }

    class Stop extends InstantCommand {

        Stop() {
            requires(DriveTrain.this);
        }

        @Override
        protected void initialize() {
            drive.set(DriveMode.PercentOutput, 0, 0);
        }
    }

    class Mecanum extends Command {
        private final double forward;
        private final double turn;
        private final double strafe;

        Mecanum(double forward, double turn, double strafe) {
            this.forward = forward;
            this.turn = turn;
            this.strafe = strafe;
            requires(DriveTrain.this);
        }

        public Mecanum(double forward, double turn) {
            this(forward, turn, 0);
        }

        @Override
        protected void execute() {
            drive.set(DriveMode.PercentOutput,
                    forward * proportionalLateralSpeed,
                    turn * proportionalTurnSpeed,
                    strafe * proportionalLateralSpeed);
        }

        @Override
        protected boolean isFinished() {
            return false;
        }
    }
}
