package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Robot extends TimedRobot {

    private final Joystick driveStick = new Joystick(0);
    private final Joystick controlStick = new Joystick(1);

    private final DriveTrain driveTrain = new DriveTrain();
    private final ArmStage1 armStage1 = new ArmStage1();
    private final ArmStage2 armStage2 = new ArmStage2();
    private final ArmTilt armTilt = new ArmTilt();
    private final Manipulator manipulator = new Manipulator();
    private final Winch winch = new Winch();
    private final Roller roller = new Roller();

    private final String NoActionAutoString = "Do Nothing";
    private final String OneFootWonderAutoString = "One Foot Wonder";
    private final String DriveToAutoLineAutoString = "Drive to Auto Line";
    private final String DriveToNullTerritoryAutoString = "Drive to Null Territory";

    private final ArrayList<String> autoStrings = new ArrayList<>();
    {
        autoStrings.add(NoActionAutoString);
        autoStrings.add(OneFootWonderAutoString);
        autoStrings.add(DriveToAutoLineAutoString);
        autoStrings.add(DriveToNullTerritoryAutoString);
    }

    public Robot() {
        driveStick.setTwistChannel(3);
        controlStick.setTwistChannel(3);
        driveStick.setThrottleChannel(2);
        driveStick.setThrottleChannel(2);
    }

    @Override
    public void robotInit(){
        CameraServer.getInstance().startAutomaticCapture();
        SmartDashboard.putStringArray("Auto List",autoStrings.toArray(new String[autoStrings.size()]));
    }

    @Override
    public void teleopInit() {
        new JoystickButton(controlStick, 1).whileActive(manipulator.gripper.new Close());
        new JoystickButton(controlStick, 2).whileActive(manipulator.gripper.new Open());

        new JoystickButton(controlStick, 3).whileActive(armStage1.new Extend());
        new JoystickButton(controlStick, 5).whileActive(armStage1.new Retract());


        new JoystickButton(controlStick, 4).whileActive(manipulator.belt.new Out());
        new JoystickButton(controlStick, 6).whileActive(manipulator.belt.new In());

        new JoystickButton(driveStick, 4).whileActive(armTilt.new Raise());
        new JoystickButton(driveStick, 6).whileActive(armTilt.new Lower());

        new JoystickButton(driveStick, 2).whileActive(winch.new Retract());

        new JoystickButton(driveStick, 3).whileActive(roller.new Out());
        new JoystickButton(driveStick, 5).whileActive(roller.new In());
    }

    @Override
    public void teleopPeriodic() {



        if (driveStick.getTrigger()) {
            driveTrain.new Drive(
                    Math.pow(-driveStick.getY() / 2,3),
                    Math.pow(driveStick.getTwist() / 2,3),
                    Math.pow(driveStick.getX()/2,3)).start();
        } else {
            driveTrain.new Drive(
                    Math.pow(-driveStick.getY(),3),
                    Math.pow(driveStick.getTwist(),3),
                    Math.pow(driveStick.getX(),3)).start();
        }

        armStage2.new Proportional(-controlStick.getY()).start();

        Scheduler.getInstance().run();
    }

    private boolean firstRun = true;

    @Override
    public void disabledInit() {
        if (firstRun) {
            new CommandGroup() {
                {
                    addSequential(new FullStop());
                    addSequential(driveTrain.new CalibrateGyro());
                    setRunWhenDisabled(true);
                }
            }.start();
            firstRun = false;
        } else new FullStop().start();
    }

    @Override
    public void disabledPeriodic(){
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        String autoString = SmartDashboard.getString("Auto Selector","");
        switch (autoString) {
            case NoActionAutoString:
                new FullStop().start();
                break;
            case DriveToAutoLineAutoString:
                new DriveToAutoLine().start();
                break;
            case DriveToNullTerritoryAutoString:
                new DriveToCenterOfNullTerritory().start();
                break;
            case OneFootWonderAutoString:
                driveTrain.new DriveEncoder(1).start();
                break;
        }
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    class FullStop extends CommandGroup {
        FullStop() {
            addParallel(driveTrain.new Stop());
            addParallel(armTilt.new Stop());
            addParallel(armStage1.new Stop());
            addParallel(armStage2.new Stop());
            addParallel(manipulator.new Stop());
            addParallel(winch.new Stop());
            setRunWhenDisabled(true);
        }
    }

    private static final double FieldLength = 54;
    private static final double AutoLine = 10;
    private static final double RobotLength = 29.0 / 12;
    private static final double AllianceStationWidth = 22;
    private static final double AllianceStationToSwitch = 14;
    private static final double SwitchDepth = 4 + 8.0 / 12;
    private static final double SwitchWidth = 12 + 9.5 / 12;

    class DriveToAutoLine extends CommandGroup {
        DriveToAutoLine() {
            //add 8 inch margin to ensure crossing
            addSequential(driveTrain.new DriveEncoder(AutoLine - RobotLength + 8.0 / 12));
        }
    }

    class DriveToCenterOfNullTerritory extends CommandGroup {
        DriveToCenterOfNullTerritory() {
            addSequential(driveTrain.new DriveEncoder(FieldLength / 2 - RobotLength / 2));
        }
    }

    @SuppressWarnings("unused")
    class PutCubeInSwitch extends CommandGroup {
        PutCubeInSwitch(SwitchPosition switchPosition, StartingPosition startingPosition) {
            if ((switchPosition == SwitchPosition.BlueLeft && DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue
                    && startingPosition == StartingPosition.Left) ||
                    (switchPosition == SwitchPosition.BlueLeft && DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Red
                            && startingPosition == StartingPosition.Right) ||
                    (switchPosition == SwitchPosition.BlueRight && DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue
                            && startingPosition == StartingPosition.Right) ||
                    (switchPosition == SwitchPosition.BlueRight && DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Red
                            && startingPosition == StartingPosition.Left)) {
                //robot starts on same side as switch
                System.out.println("Robot starts on same side as switch target");

                //drive forward until in line with switch
                addSequential(driveTrain.new DriveEncoder(AllianceStationToSwitch + SwitchDepth / 2));

                //turn toward switch
                if (startingPosition == StartingPosition.Left) {
                    addSequential(driveTrain.new TurnByAmount(90));
                } else { //Right
                    addSequential(driveTrain.new TurnByAmount(-90));
                }
                //drive to switch
                addSequential(driveTrain.new DriveEncoder(AllianceStationWidth / 2 - SwitchWidth / 2 - RobotLength, 0.2));

                //put cube in box
                addSequential(new PutCubeIn());

            } else if ((switchPosition == SwitchPosition.BlueLeft && DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue
                    && startingPosition == StartingPosition.Right) ||
                    (switchPosition == SwitchPosition.BlueLeft && DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Red
                            && startingPosition == StartingPosition.Left) ||
                    (switchPosition == SwitchPosition.BlueRight && DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue
                            && startingPosition == StartingPosition.Left) ||
                    (switchPosition == SwitchPosition.BlueRight && DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Red
                            && startingPosition == StartingPosition.Right)) {
                //robot starts on opposite side of switch
                System.out.println("Robot starts on opposite side as switch target");

                //drive past switch
                addSequential(driveTrain.new DriveEncoder(AllianceStationWidth + SwitchDepth * 3 / 2));
                //drive behind switch
                if (startingPosition == StartingPosition.Left) {
                    addSequential(driveTrain.new TurnByAmount(90));
                } else { //Right
                    addSequential(driveTrain.new TurnByAmount(-90));
                }
                addSequential(driveTrain.new DriveEncoder(AllianceStationWidth-RobotLength));
                //drive back to in-line with switch
                if (startingPosition == StartingPosition.Left) {
                    addSequential(driveTrain.new TurnByAmount(90));
                } else { //Right
                    addSequential(driveTrain.new TurnByAmount(-90));
                }
                addSequential(driveTrain.new DriveEncoder(SwitchDepth));
                //point toward switch
                if (startingPosition == StartingPosition.Left) {
                    addSequential(driveTrain.new TurnByAmount(90));
                } else { //Right
                    addSequential(driveTrain.new TurnByAmount(-90));
                }
                //drive to switch
                addSequential(driveTrain.new DriveEncoder(AllianceStationWidth / 2 - SwitchWidth / 2 - RobotLength, 0.2));

                //put cube in box
                addSequential(new PutCubeIn());
            } else {
                System.err.println("This combination of starting position, team, and switch position has no corresponding routine");
            }
        }

        private class PutCubeIn extends CommandGroup {
            PutCubeIn() {
                addSequential(armStage1.new Extend() {
                    {
                        setTimeout(1);
                    }

                    @Override
                    protected boolean isFinished() {
                        return super.isFinished() || isTimedOut();
                    }
                });
                addSequential(armTilt.new Raise() {
                    {
                        setTimeout(1);
                    }

                    @Override
                    protected boolean isFinished() {
                        return super.isFinished() || isTimedOut();
                    }
                });
                addSequential(manipulator.gripper.new Open() {
                    {
                        setTimeout(0.25);
                    }

                    @Override
                    protected boolean isFinished() {
                        return super.isFinished() || isTimedOut();
                    }
                });
            }
        }
    }

    enum SwitchPosition {
        BlueLeft,
        BlueRight
    }

    enum StartingPosition {
        Left,
        Right,
        @SuppressWarnings("unused") Center
    }
}
