package com.robocubs4205.frc2018;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


@SuppressWarnings("FieldCanBeLocal")
class DriveTrain extends Subsystem {
    private WPI_TalonSRX frontLeft = new WPI_TalonSRX(13);
    private WPI_TalonSRX frontRight = new WPI_TalonSRX(11);
    private WPI_TalonSRX rearLeft = new WPI_TalonSRX(12);
    private WPI_TalonSRX rearRight = new WPI_TalonSRX(10);

    private final DifferentialDrive drive = new DifferentialDrive(
            new SpeedControllerGroup(frontLeft, rearLeft),
            new SpeedControllerGroup(frontRight, rearRight));

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private final int CPR = 4096;

    private final double wheelDiameter = (5f+3f/8)/12;
    private final double wheelCircumference = wheelDiameter * Math.PI;
    private final int CPF = (int) (CPR / wheelCircumference);

    private final double proportionalLateralPower = 1;
    private final double proportionalTurnPower = 1;

    DriveTrain() {
        rearLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rearRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rearRight.setInverted(true);
        frontRight.setInverted(true);
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
            drive.arcadeDrive(0, 0);
        }
    }

    class Arcade extends PerpetualCommand {
        private final double forward;
        private final double turn;

        Arcade(double forward, double turn) {
            this.forward = forward;
            this.turn = turn;
            requires(DriveTrain.this);
        }

        @Override
        protected void execute() {
            drive.arcadeDrive(
                    forward * proportionalLateralPower,
                    turn * proportionalTurnPower);
        }
    }

    class DriveEncoder extends Command {
        private final double distance;
        private final double power;
        private final double tolerance;

        /**
         * Drive forward a specific distance
         *
         * @param distance the distance in feet
         */
        DriveEncoder(double distance) {
            this(distance, 0.5);
        }

        /**
         * Drive forward a specific distance
         *
         * @param distance  the distance in feet
         * @param power     the motor power on the range (0,1]
         */
        DriveEncoder(double distance, double power){
            this(distance, power,2f/12);
        }

        /**
         * Drive forward a specific distance
         *
         * @param distance  the distance in feet
         * @param power     the motor power on the range (0,1]
         * @param tolerance the tolerance in feet within which the command will finish
         */
        DriveEncoder(double distance, double power, double tolerance) {
            requires(DriveTrain.this);
            this.distance = distance;
            this.power = power;
            this.tolerance = tolerance;
        }

        @Override
        protected void initialize() {
            rearLeft.setSelectedSensorPosition(0, 0, 10);
            rearRight.setSelectedSensorPosition(0, 0, 10);
            rearLeft.config_kP(0, 0.125, 10);
            rearRight.config_kP(0, 0.125, 10);
            rearLeft.configPeakOutputForward(power, 10);
            rearRight.configPeakOutputForward(power, 10);
            rearLeft.configPeakOutputReverse(-power, 10);
            rearRight.configPeakOutputReverse(-power, 10);
            rearLeft.setSensorPhase(true);
            rearRight.setSensorPhase(false);

            frontLeft.follow(rearLeft);
            frontRight.follow(rearRight);
        }

        @Override
        protected void execute() {
            rearLeft.set(ControlMode.Position, CPF * distance);
            rearRight.set(ControlMode.Position, CPF * distance);
        }

        @Override
        protected boolean isFinished() {
            //getClosedLoopError not used since it sometimes erroneously reports 0
            return (Math.abs(rearLeft.getSelectedSensorPosition(0)-CPF*distance) < CPF * tolerance)&&
                    (Math.abs(rearRight.getSelectedSensorPosition(0)-CPF*distance) < CPF * tolerance);
        }
    }

    class TurnByAmount extends PIDCommand {

        private final double tolerance;

        TurnByAmount(double angle){
            this(angle,5);
        }

        TurnByAmount(double angle, double tolerance){
            super(1f/20,0,0);
            this.tolerance = tolerance;
            requires(DriveTrain.this);
            getPIDController().setContinuous();
            getPIDController().setInputRange(0,360);
            getPIDController().setOutputRange(-0.3,0.3);
            setSetpoint(gyro.getAngle()+angle);
        }

        @Override
        protected boolean isFinished() {
            return Math.abs(getPIDController().getError())< tolerance;
        }

        @Override
        protected double returnPIDInput() {
            return gyro.getAngle();
        }

        @Override
        protected void usePIDOutput(double output) {
            drive.arcadeDrive(0,output);
        }
    }

    class CalibrateGyro extends InstantCommand{
        CalibrateGyro(){
            setRunWhenDisabled(true);
        }
        @Override
        protected void execute(){
            gyro.calibrate();
        }
    }
}
