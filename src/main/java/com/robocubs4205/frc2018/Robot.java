package com.robocubs4205.frc2018;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot{

    private Joystick driveStick = new Joystick(0);
    private Joystick controlStick = new Joystick(1);

    private DriveTrain driveTrain = new DriveTrain();
    private ArmStage1 armStage1 = new ArmStage1();
    private ArmStage2 armStage2 = new ArmStage2();
    private ArmTilt armTilt = new ArmTilt();
    private Gripper gripper = new Gripper();
    private Winch winch = new Winch();

    @Override
    public void teleopInit(){
        new JoystickButton(controlStick,1).whileActive(gripper.new Close());
        new JoystickButton(controlStick,2).whileActive(gripper.new Open());

        new JoystickButton(controlStick,3).whileActive(armStage1.new Extend());
        new JoystickButton(controlStick,5).whileActive(armStage1.new Retract());

        new JoystickButton(controlStick,4).whileActive(armTilt.new Raise());
        new JoystickButton(controlStick,6).whileActive(armTilt.new Lower());

        new JoystickButton(driveStick,2).whileActive(winch.new Retract());
    }

    @Override
    public void teleopPeriodic(){

        if(driveStick.getTrigger()){
            driveTrain.new Mecanum(-driveStick.getY(),driveStick.getX(),driveStick.getTwist()).start();
        }
        else {
            driveTrain.new Mecanum(-driveStick.getY()/2,driveStick.getX()/2,driveStick.getTwist()).start();
        }

        armStage2.new Proportional(-controlStick.getY()).start();

        Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit(){
      new FullStop().start();
    }

    class FullStop extends CommandGroup{
        FullStop(){
            addParallel(driveTrain.new Stop());
            addParallel(armTilt.new Stop());
            addParallel(armStage1.new Stop());
            addParallel(armStage2.new Stop());
            addParallel(gripper.new Stop());
            addParallel(winch.new Stop());
        }
    }
}
