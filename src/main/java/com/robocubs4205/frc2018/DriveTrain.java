package com.robocubs4205.frc2018;

import com.ctre.phoenix.drive.DriveMode;
import com.ctre.phoenix.drive.MecanumDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.*;


@SuppressWarnings("FieldCanBeLocal")
class DriveTrain extends Subsystem {
    private TalonSRX frontLeft = new TalonSRX(13);
    private TalonSRX frontRight = new TalonSRX(11);
    private TalonSRX rearLeft = new TalonSRX(12);
    private TalonSRX rearRight = new TalonSRX(10);

    private final MecanumDrive drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private final int CPR = 4096;

    private final double wheelDiameter = (5f+3f/8)/12;
    private final double wheelCircumference = wheelDiameter * Math.PI;
    private final int CPF = (int) (CPR / wheelCircumference);

    private final double proportionalLateralSpeed = 1;
    private final double proportionalTurnSpeed = 1;

    DriveTrain() {
        rearLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rearRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
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

    class Mecanum extends PerpetualCommand {
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
    }

    class DriveEncoder extends Command {
        private final double distance;
        private final double speed;
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
         * @param speed     the speed on the range (0,1]
         */
        DriveEncoder(double distance, double speed){
            this(distance,speed,2f/12);
        }

        /**
         * Drive forward a specific distance
         *
         * @param distance  the distance in feet
         * @param speed     the speed on the range (0,1]
         * @param tolerance the tolerance in feet within which the command will finish
         */
        DriveEncoder(double distance, double speed, double tolerance) {
            requires(DriveTrain.this);
            this.distance = distance;
            this.speed = speed;
            this.tolerance = tolerance;
        }

        @Override
        protected void initialize() {
            rearLeft.setSelectedSensorPosition(0, 0, 10);
            rearRight.setSelectedSensorPosition(0, 0, 10);
            rearLeft.config_kP(0, 0.125, 10);
            rearRight.config_kP(0, 0.125, 10);
            rearLeft.configPeakOutputForward(speed, 10);
            rearRight.configPeakOutputForward(speed, 10);
            rearLeft.configPeakOutputReverse(-speed, 10);
            rearRight.configPeakOutputReverse(-speed, 10);
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
            getPIDController().setOutputRange(-1,1);
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
            drive.set(DriveMode.PercentOutput,0,output,0);
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
