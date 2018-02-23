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
    private final String AdvancedAuto = "Advanced (Uses settings in \"Auto Config\" tab)";

    private final ArrayList<String> autoStrings = new ArrayList<>();

    {
        autoStrings.add(NoActionAutoString);
        autoStrings.add(OneFootWonderAutoString);
        autoStrings.add(DriveToAutoLineAutoString);
        autoStrings.add(DriveToNullTerritoryAutoString);
        autoStrings.add(AdvancedAuto);
    }

    public Robot() {
        driveStick.setTwistChannel(3);
        controlStick.setTwistChannel(3);
        driveStick.setThrottleChannel(2);
        driveStick.setThrottleChannel(2);
    }

    @Override
    public void robotInit() {
        CameraServer.getInstance().startAutomaticCapture();
        SmartDashboard.putStringArray("Auto List", autoStrings.toArray(new String[autoStrings.size()]));
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
                    Math.pow(-driveStick.getY() / 2, 3),
                    Math.pow(driveStick.getTwist() / 2, 3),
                    Math.pow(driveStick.getX() / 2, 3)).start();
        } else {
            driveTrain.new Drive(
                    Math.pow(-driveStick.getY(), 3),
                    Math.pow(driveStick.getTwist(), 3),
                    Math.pow(driveStick.getX(), 3)).start();
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
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        String autoString = SmartDashboard.getString("Auto Selector", "");
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
            case AdvancedAuto:
                initAdvancedAuto();
                break;
        }
    }

    private final int StartingPositionNotFound = -1;
    private final int StartingPositionLeft = 0;
    private final int StartingPositionCenterLeft = 1;
    private final int StartingPositionCenter = 2;
    private final int StartingPositionCenterRight = 3;
    private final int StartingPositionRight = 4;

    private final int SwitchSameSideActionNotFound = -1;
    private final int SwitchSameSideActionNothing = 0;
    private final int SwitchSameSideActionProceed = 1;
    private final int SwitchSameSideActionScale = 3;
    private final int SwitchSameSideActionAutoLine = 4;
    private final int SwitchSameSideActionNullTerritory = 5;

    private final int SwitchOppositeSideActionNotFound = -1;
    private final int SwitchOppositeSideActionNothing = 0;
    private final int SwitchOppositeSideActionFrontCross = 1;
    private final int SwitchOppositeSideActionRearCross = 2;
    private final int SwitchOppositeSideActionScale = 3;
    private final int SwitchOppositeSideActionAutoLine = 4;
    private final int SwitchOppositeSideActionNullTerritory = 5;

    private final int SwitchRightSideActionNotFound = -1;
    private final int SwitchRightSideActionNothing = 0;
    private final int SwitchRightSideActionProceed = 1;

    private final int SwitchLeftSideActionNotFound = -1;
    private final int SwitchLeftSideActionNothing = 0;
    private final int SwitchLeftSideActionProceed = 1;

    private final int CubePositionNotFound = -1;
    private final int CubePositionForks = 0;
    private final int CubePositionShelf = 1;

    private void initAdvancedAuto() {
        try {
            int startingPosition = Integer.parseInt(SmartDashboard.getString("Starting Position", "-1"));
            int switchOppositeSideAction =
                    Integer.parseInt(SmartDashboard.getString("Switch OppositeSideAction", "-1"));
            int switchSameSideAction =
                    Integer.parseInt(SmartDashboard.getString("Switch Same Side Action", "-1"));
            int switchRightSideAction =
                    Integer.parseInt(SmartDashboard.getString("Switch Right Side Action", "-1"));
            int switchLeftSideAction =
                    Integer.parseInt(SmartDashboard.getString("Switch Left Side Action", "-1"));
            int switchCubePosition =
                    Integer.parseInt(SmartDashboard.getString("Cube Position", "-1"));
            if (startingPosition == StartingPositionNotFound ||
                    (startingPosition != StartingPositionCenter &&
                            (switchOppositeSideAction == SwitchOppositeSideActionNotFound ||
                                    switchSameSideAction == SwitchSameSideActionNotFound)) ||
                    (startingPosition == StartingPositionCenter &&
                            (switchRightSideAction == SwitchRightSideActionNotFound ||
                                    switchLeftSideAction == SwitchLeftSideActionNotFound))) {
                System.err.println("Received incomplete autonomous configuration from drive station. Aborting.");
                new Throwable().printStackTrace();
                DriverStation.reportError("Recieved incomplete autonomous configuration from drive station. Aborting", false);
                return;
            }

            SwitchCubePosition switchCubePositionEnum;
            switch (switchCubePosition) {
                case CubePositionForks:
                    switchCubePositionEnum = SwitchCubePosition.Forks;
                    break;
                case CubePositionShelf:
                    switchCubePositionEnum = SwitchCubePosition.Shelf;
                default:
                    System.err.println("Autonomous case not implemented. Aborting.");
                    new Throwable().printStackTrace();
                    DriverStation.reportError("Autonomous case not implemented. Aborting.", false);
                    return;
            }

            SwitchPosition switchPosition = SwitchPosition.Left;
            String gameData = DriverStation.getInstance().getGameSpecificMessage();
            if (gameData.length() != 3) {
                System.err.println("Invalid game data received from FMS. Aborting.");
                new Throwable().printStackTrace();
                DriverStation.reportError("Invalid game data received from FMS. Aborting.", false);
            } else if (gameData.charAt(0) == 'L') switchPosition = SwitchPosition.Left;
            else if (gameData.charAt(0) == 'R') switchPosition = SwitchPosition.Right;
            else {
                System.err.println("Invalid game data received from FMS. Aborting.");
                new Throwable().printStackTrace();
                DriverStation.reportError("Invalid game data received from FMS. Aborting.", false);
            }

            switch (startingPosition) {
                case StartingPositionLeft:
                    if (switchPosition == SwitchPosition.Left) {
                        if (switchSameSideAction == SwitchSameSideActionProceed)
                            new SwitchAutoSameSideFarLeftOrRight(StartingPosition.Left, switchCubePositionEnum).start();
                        else if (switchSameSideAction == SwitchSameSideActionScale)
                            new ScaleAutoFarLeftOrRight(StartingPosition.Left);
                        else if(switchSameSideAction == SwitchSameSideActionAutoLine)
                            new DriveToAutoLine().start();
                        else if (switchSameSideAction == SwitchSameSideActionNullTerritory)
                            new DriveToCenterOfNullTerritory().start();
                        else if (switchSameSideAction != SwitchSameSideActionNothing) {
                            System.err.println("Autonomous case not implemented. Aborting.");
                            new Throwable().printStackTrace();
                            DriverStation.reportError("Autonomous case not implemented. Aborting.", false);
                        }
                    } else {
                        if (switchOppositeSideAction == SwitchOppositeSideActionRearCross)
                            new SwitchAutoOppositeSideCrossBehind(StartingPosition.Left, switchCubePositionEnum).start();
                        else if (switchOppositeSideAction == SwitchOppositeSideActionScale)
                            new ScaleAutoFarLeftOrRight(StartingPosition.Left);
                        else if(switchOppositeSideAction == SwitchOppositeSideActionAutoLine)
                            new DriveToAutoLine().start();
                        else if (switchOppositeSideAction == SwitchOppositeSideActionNullTerritory)
                            new DriveToCenterOfNullTerritory().start();
                        else if (switchOppositeSideAction != SwitchOppositeSideActionNothing) {
                            System.err.println("Autonomous case not implemented. Aborting.");
                            new Throwable().printStackTrace();
                            DriverStation.reportError("Autonomous case not implemented. Aborting.", false);
                        }
                    }
                    break;
                case StartingPositionRight:
                    if (switchPosition == SwitchPosition.Right) {
                        if (switchSameSideAction == SwitchSameSideActionProceed)
                            new SwitchAutoSameSideFarLeftOrRight(StartingPosition.Right, switchCubePositionEnum).start();
                        else if (switchSameSideAction == SwitchSameSideActionScale)
                            new ScaleAutoFarLeftOrRight(StartingPosition.Right);
                        else if (switchSameSideAction != SwitchSameSideActionNothing) {
                            System.err.println("Autonomous case not implemented. Aborting.");
                            new Throwable().printStackTrace();
                            DriverStation.reportError("Autonomous case not implemented. Aborting.", false);
                        }
                    } else {
                        if (switchOppositeSideAction == SwitchOppositeSideActionRearCross)
                            new SwitchAutoOppositeSideCrossBehind(StartingPosition.Right, switchCubePositionEnum).start();
                        else if (switchOppositeSideAction == SwitchOppositeSideActionScale)
                            new ScaleAutoFarLeftOrRight(StartingPosition.Right);
                        else if (switchOppositeSideAction != SwitchOppositeSideActionNothing) {
                            System.err.println("Autonomous case not implemented. Aborting.");
                            new Throwable().printStackTrace();
                            DriverStation.reportError("Autonomous case not implemented. Aborting.", false);
                        }
                    }
                    break;
                case StartingPositionCenterLeft:
                    if (switchPosition == SwitchPosition.Left) {
                        if (switchSameSideAction == SwitchSameSideActionProceed)
                            new SwitchAutoSameSideCenterLeftOrRight(switchCubePositionEnum);
                        else if (switchSameSideAction != SwitchSameSideActionNothing) {
                            System.err.println("Autonomous case not implemented. Aborting.");
                            new Throwable().printStackTrace();
                            DriverStation.reportError("Autonomous case not implemented. Aborting.", false);
                        }
                    } else System.err.println("Autonomous case not implemented. Aborting.");
                    break;
                case StartingPositionCenterRight:
                    if (switchPosition == SwitchPosition.Right) {
                        if (switchSameSideAction == SwitchSameSideActionProceed)
                            new SwitchAutoSameSideCenterLeftOrRight(switchCubePositionEnum);
                        else if (switchSameSideAction != SwitchSameSideActionNothing) {
                            System.err.println("Autonomous case not implemented. Aborting.");
                            new Throwable().printStackTrace();
                            DriverStation.reportError("Autonomous case not implemented. Aborting.", false);
                        }
                    } else System.err.println("Autonomous case not implemented. Aborting.");
                    break;
                default:
                    System.err.println("Autonomous case not implemented. Aborting.");
            }
        }
        catch (NumberFormatException e){
            System.err.println("Invalid data received from the Driver Station. Aborting.");
            e.printStackTrace();
            DriverStation.reportError("Invalid data received from the Driver Station. Aborting.",false);
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
    private static final double FieldWidth = 27;
    private static final double AutoLine = 10;
    private static final double RobotBumperLength = 34.0 / 12;
    private static final double AllianceStationWidth = 22;
    private static final double AllianceStationToSwitch = 14;
    private static final double SwitchDepth = 4 + 8.0 / 12;
    private static final double SwitchWidth = 12 + 9.5 / 12;
    private static final double ScaleWidth = 15;

    class DriveToAutoLine extends CommandGroup {
        DriveToAutoLine() {
            //add 8 inch margin to ensure crossing
            addSequential(driveTrain.new DriveEncoder(AutoLine - RobotBumperLength + 8.0 / 12));
        }
    }

    class DriveToCenterOfNullTerritory extends CommandGroup {
        DriveToCenterOfNullTerritory() {
            addSequential(driveTrain.new DriveEncoder(FieldLength / 2 - RobotBumperLength / 2));
        }
    }

    enum SwitchPosition {
        Left,
        Right
    }

    enum SwitchCubePosition {
        Forks,
        Shelf
    }

    private class SwitchAutoSameSideFarLeftOrRight extends CommandGroup {
        SwitchAutoSameSideFarLeftOrRight(StartingPosition startingPosition, SwitchCubePosition cubePosition) {
            //drive forward until in line with switch
            {
                double distance = AllianceStationToSwitch + SwitchDepth / 2;
                switch (cubePosition) {
                    case Forks:
                        addSequential(driveTrain.new DriveEncoder(distance));
                        return;
                    case Shelf:
                        addSequential(driveTrain.new DriveEncoder(-distance));
                }
            }


            //turn toward switch
            if (startingPosition == StartingPosition.Left) {
                addSequential(driveTrain.new TurnByAmount(90));
            } else { //Right
                addSequential(driveTrain.new TurnByAmount(-90));
            }
            //drive to switch
            {
                double distance = AllianceStationWidth / 2 - SwitchWidth / 2 - RobotBumperLength;
                switch (cubePosition) {
                    case Forks:
                        addSequential(driveTrain.new DriveEncoder(distance, 0.2));
                        return;
                    case Shelf:
                        addSequential(driveTrain.new DriveEncoder(-distance, 0.2));
                }
            }

            //put cube in box
            switch (cubePosition) {
                case Forks:
                    addSequential(armStage1.new Extend(),1);
                    addSequential(manipulator.belt.new Out(),1);
            }

        }
    }

    private class SwitchAutoOppositeSideCrossBehind extends CommandGroup {
        SwitchAutoOppositeSideCrossBehind(StartingPosition startingPosition, SwitchCubePosition cubePosition) {
            System.out.println("Robot starts on opposite side as switch target");

            //drive past switch
            {
                double distance = AllianceStationWidth + SwitchDepth * 3 / 2;
                switch (cubePosition){
                    case Forks:
                        addSequential(driveTrain.new DriveEncoder(distance));
                    case Shelf:
                        addSequential(driveTrain.new DriveEncoder(-distance));
                }
            }
            //drive behind switch
            if (startingPosition == StartingPosition.Left) {
                addSequential(driveTrain.new TurnByAmount(90));
            } else { //Right
                addSequential(driveTrain.new TurnByAmount(-90));
            }
            {
                double distance = AllianceStationWidth - RobotBumperLength;
                switch (cubePosition){
                    case Forks:
                        addSequential(driveTrain.new DriveEncoder(distance));
                    case Shelf:
                        addSequential(driveTrain.new DriveEncoder(-distance));
                }
            }
            //drive back to in-line with switch
            if (startingPosition == StartingPosition.Left) {
                addSequential(driveTrain.new TurnByAmount(90));
            } else { //Right
                addSequential(driveTrain.new TurnByAmount(-90));
            }
            {
                double distance = SwitchDepth;
                switch (cubePosition){
                    case Forks:
                        addSequential(driveTrain.new DriveEncoder(distance));
                    case Shelf:
                        addSequential(driveTrain.new DriveEncoder(-distance));
                }
            }
            //point toward switch
            if (startingPosition == StartingPosition.Left) {
                addSequential(driveTrain.new TurnByAmount(90));
            } else { //Right
                addSequential(driveTrain.new TurnByAmount(-90));
            }
            //drive to switch
            {
                double distance = AllianceStationWidth / 2 - SwitchWidth / 2 - RobotBumperLength;
                switch (cubePosition){
                    case Forks:
                        addSequential(driveTrain.new DriveEncoder(distance));
                    case Shelf:
                        addSequential(driveTrain.new DriveEncoder(-distance));
                }
            }
            //put cube in box
            addSequential(armStage1.new Extend(),1);
            addSequential(manipulator.belt.new Out(),1);
        }
    }

    private class SwitchAutoSameSideCenterLeftOrRight extends CommandGroup {
        SwitchAutoSameSideCenterLeftOrRight(SwitchCubePosition cubePosition) {

            if (cubePosition == SwitchCubePosition.Forks) addSequential(armStage1.new Extend(),1);
            {
                double distance = AllianceStationToSwitch - RobotBumperLength;
                switch (cubePosition){
                    case Forks:
                        addSequential(driveTrain.new DriveEncoder(distance));
                    case Shelf:
                        addSequential(driveTrain.new DriveEncoder(-distance));
                }
            }
            if (cubePosition == SwitchCubePosition.Forks) addSequential(manipulator.belt.new Out(),1);
            else if (cubePosition == SwitchCubePosition.Shelf) addSequential(roller.new Out());
        }
    }

    private class ScaleAutoFarLeftOrRight extends CommandGroup {
        /**
         * Fork only
         */
        ScaleAutoFarLeftOrRight(StartingPosition startingPosition) {
            addSequential(driveTrain.new DriveEncoder(FieldLength / 2));
            addParallel(armStage1.new Extend(), 8);
            addParallel(armStage2.new Proportional(1), 8);
            if (startingPosition == StartingPosition.Left) {
                addSequential(driveTrain.new TurnByAmount(90));
            } else {
                addSequential(driveTrain.new TurnByAmount(-90));
            }
            addSequential(driveTrain.new DriveEncoder(AllianceStationWidth / 2 - ScaleWidth / 2 - RobotBumperLength, 0.2));
            addSequential(manipulator.belt.new Out(), 1);
        }
    }

    enum StartingPosition {
        Left,
        Right,
        @SuppressWarnings("unused") Center
    }
}
