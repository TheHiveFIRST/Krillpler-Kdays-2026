package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
  private final VisionSubsystem mVisionSubsystem = new VisionSubsystem(); 
  private final DriveSubsystem mDriveSubsystem = new DriveSubsystem(); 
  private final HopperSubsystem mhoppersubsystem = new HopperSubsystem();
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem(); 
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem(); 
  private final TurretSubsystem mTurretSubsystem = new TurretSubsystem();
  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController mDriverController = 
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController mOperatorController = 
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER);
  
      boolean slowMode = false;


  public RobotContainer() {
    
    

    

   



    autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser).withSize(2, 1);
    
    
    configureBindings();
    mDriveSubsystem.setDefaultCommand(  
    new RunCommand(
            () -> {
            double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
            mDriveSubsystem.driveJoystick(
                MathUtil.applyDeadband(-mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(-mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                true);},
            mDriveSubsystem));

       
        // set default commands here
        
        
     }

    public void configureJoysticks(String pickedAuto) {
      switch (pickedAuto) {
        case "DOUBLE_SWIPE_HUMAN_PLAYER":
        case "SINGLE_SWIPE_HUMAN_PLAYER":
          DriveSubsystem.gyrooffset = -90;
          mDriveSubsystem.setDefaultCommand(
          new RunCommand(() -> {
              double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
              mDriveSubsystem.driveJoystick(
                  MathUtil.applyDeadband(-mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  true);},
              mDriveSubsystem));
          
          break;
        case "DOUBLE_SWIPE_DEPOT":
        case "SINGLE_SWIPE_DEPOT":
          DriveSubsystem.gyrooffset = 90;
          mDriveSubsystem.setDefaultCommand(
          new RunCommand(() -> {
              double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
              mDriveSubsystem.driveJoystick(
                  MathUtil.applyDeadband(-mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  true);},
              mDriveSubsystem));
          break;
        default: // start in front of the hub
          DriveSubsystem.gyrooffset = 0;
          mDriveSubsystem.setDefaultCommand(  
            new RunCommand(() -> {
              double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
              mDriveSubsystem.driveJoystick(
                  MathUtil.applyDeadband(-mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  true);},
              mDriveSubsystem));
          break;
      }
    }
    private void configureBindings() {
      
        //OPERATOR CONTROLS
        mOperatorController.y().whileTrue(mShooterSubsystem.toggleShooterCommand());
        
        
        mOperatorController.rightBumper().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mOperatorController.leftBumper().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());

        mOperatorController.povDown().whileTrue(mDriveSubsystem.characterizeAngular(3));
        mOperatorController.povUp().whileTrue(mDriveSubsystem.characterizeLinear(4));


        //DRIVER CONTROLS
        mDriverController.y().whileTrue(mShooterSubsystem.toggleShooterCommand()); 
        
        

        
     
        
 
        mDriverController.start().whileTrue(mDriveSubsystem.resetGyro()); 
        

        /*mDriverController.povLeft().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mDriverController.povRight().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());
        mDriverController.povDown().onTrue(toggleSlowMode());
        mDriverController.povUp().whileTrue(mIntakeSubsystem.runIntakeSlowCommand());*/
       // mDriverController.povUp().onTrue(mShooterSubsystem.runOnce(mDriveSubsystem::incrementPalign));
       // mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mDriveSubsystem::decrementPalign));
        //mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));


        // tag based autoalign, useful for pick and place with alignment in x and y 
        //mDriverController.a().whileTrue(new RunCommand(
        //  () -> mDriveSubsystem.driveJoystick(
        //    MathUtil.applyDeadband(-mDriverController.getLeftY(), OperatorConstants.DRIVE_DEADBAND),
        //    MathUtil.applyDeadband(-mDriverController.getLeftX(), OperatorConstants.DRIVE_DEADBAND),
        //   -mVisionSubsystem.autoAlignRotationSpeed(), 
        //   true), mDriveSubsystem));
    

       
        //PID TUNING
        // Back Button
       //mDriverController.back()
           // .whileTrue(mShooterSubsystem.run(mShooterSubsystem::cycleTuningMode))
           // .debounce(0.3); // Prevents accidental double presses
        
        //mDriverController.start().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementCurrentGain));
        //mDriverController.back().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementCurrentGain));
        //mDriverController.povUp().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementKP));
        //mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementKP));

    }

    public void zeroGyroHeading(){
      mDriveSubsystem.zeroHeading();
    }

    
    
    public Command toggleSlowMode(){
        return new InstantCommand(() -> slowMode = !slowMode);
    }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
  

    
  

   
}

  


