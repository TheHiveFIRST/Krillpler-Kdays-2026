package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomousTurretOperationCommand;
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

       
        mIntakeSubsystem.setDefaultCommand(mIntakeSubsystem.stopIntakeCommand());
        mhoppersubsystem.setDefaultCommand(mhoppersubsystem.stopShootingCommand());
        mShooterSubsystem.setDefaultCommand(mShooterSubsystem.runShooterPIDFCommand());
        mTurretSubsystem.setDefaultCommand(runTurretAutomatically(mTurretSubsystem, mDriveSubsystem));
        
        
     }

    
    private void configureBindings() {
      
        //OPERATOR CONTROLS
        //shooter toggle
        mOperatorController.y().onTrue(mShooterSubsystem.toggleShooterCommand());
        //will toggle the regression: if this is pressed then the shooter will run at hub RPM (manual adjustments are allowed)
        mOperatorController.x().onTrue(mShooterSubsystem.toggleShooterCaulculationsCommand());
        //manual turret controls
        mOperatorController.rightTrigger(0.2).whileTrue(mTurretSubsystem.runTurretClockwiseCommand());
        mOperatorController.leftTrigger(0.2).whileTrue(mTurretSubsystem.runTurretCounterClockwiseCommand());
        mOperatorController.b().onTrue(mTurretSubsystem.toggleTurretCommand());
        //manual shooting power controls
        mOperatorController.rightBumper().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mOperatorController.leftBumper().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());

        


        //DRIVER CONTROLS
        
        //the intake is deployed by default, this just runs the rollers
        mDriverController.leftBumper().whileTrue(mIntakeSubsystem.runIntakeCommand());
        
        //will enable slowmode when shooting
        mDriverController.rightBumper().whileTrue(mhoppersubsystem.runShootCommand());
        mDriverController.rightBumper().onChange(toggleSlowMode());
        
        //retracts the intake and stops the rollers
        //mDriverController.leftTrigger(0.2).whileTrue(mIntakeSubsystem.retractIntakeCommand());
        //removed because apparently we have a roller floor now
 
        
        mDriverController.x().whileTrue(mDriveSubsystem.defensePosition());
        mDriverController.start().whileTrue(mDriveSubsystem.resetGyro()); 

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

  public Command runTurretAutomatically(TurretSubsystem mTurretSubsystem, DriveSubsystem mDriveSubsystem){
    return new autonomousTurretOperationCommand(mTurretSubsystem, mDriveSubsystem);
  }
  

    
  

   
}

  


