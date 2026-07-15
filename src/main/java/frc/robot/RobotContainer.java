package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomousTurretOperationCommand;
import frc.robot.commands.shootButCheckCommand;
import frc.robot.commands.shootUnjamSequence;
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

import frc.robot.subsystems.IntakeSubsystem;




import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
  
  private final DriveSubsystem mDriveSubsystem = new DriveSubsystem(); 
  private final HopperSubsystem mhoppersubsystem = new HopperSubsystem();
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem(); // pass mDriveSubsytem in to access non-static methods getRobotRelativeSpeeds() and getPose()
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem(); 
  //private final TurretSubsystem mTurretSubsystem = new TurretSubsystem();


  private final CommandXboxController mDriverController = 
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController mOperatorController = 
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER);
  
      boolean slowMode = false;


  public RobotContainer() {
    
    

    

   



    
    
    
    configureBindings();
    mDriveSubsystem.setDefaultCommand(  
    new RunCommand(
            () -> {
            double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
            mDriveSubsystem.driveJoystick(
                MathUtil.applyDeadband(Math.pow(-mDriverController.getLeftY(), 3)*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(Math.pow(-mDriverController.getLeftX(), 3)*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                true);},
            mDriveSubsystem));

       
        mIntakeSubsystem.setDefaultCommand(mIntakeSubsystem.runIntakeCommand());
        mhoppersubsystem.setDefaultCommand(mhoppersubsystem.stopShootingCommand());
        mShooterSubsystem.setDefaultCommand(mShooterSubsystem.stopShooterCommand());
        //mTurretSubsystem.setDefaultCommand(mTurretSubsystem.stopTurretCommand());
        
        
     }

    
    private void configureBindings() {
      
        //OPERATOR CONTROLS
        //shooter toggle
        
        //will toggle the regression: if this is pressed then the shooter will run at hub RPM (manual adjustments are allowed) - hayden was here btw. thanks for the citrus sticker.
        //mOperatorController.x().onTrue(mShooterSubsystem.toggleShooterCaulculationsCommand());
        //manual turret controls
        //mOperatorController.rightTrigger(0.2).whileTrue(mTurretSubsystem.runTurretClockwiseCommand());
        //mOperatorController.leftTrigger(0.2).whileTrue(mTurretSubsystem.runTurretCounterClockwiseCommand());
        //mOperatorController.b().onTrue(mTurretSubsystem.toggleTurretCommand());
        //manual shooting power controls
        mOperatorController.leftBumper().whileTrue(mShooterSubsystem.runShooterCommand());
        

        


        //DRIVER CONTROLS
        
        //the intake is deployed by default, this just runs the rollers
        mDriverController.leftBumper().whileTrue(mIntakeSubsystem.runIntakeCommand());
        
        //will enable slowmode when shooting
        mDriverController.rightBumper().whileTrue(RunHopperUnjam().repeatedly());
        mDriverController.rightBumper().whileTrue(mShooterSubsystem.runShooterCommand());
        
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

    public Command RunHopperShoot(){
        return mhoppersubsystem.runShootCommand();
    }
    public Command RunHopperUnjam(){
        return new shootUnjamSequence(mhoppersubsystem);
    }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  //public Command runTurretAutomatically(TurretSubsystem mTurretSubsystem, DriveSubsystem mDriveSubsystem){
  //  return new autonomousTurretOperationCommand(mTurretSubsystem, mDriveSubsystem);
  //}
  
 //public Command runHopperAutomaticallyCommand(TurretSubsystem mTurretSubsystem, HopperSubsystem mHopperSubsystem, ShooterSubsystem mShooterSubsystem){
 //  return new shootButCheckCommand(mTurretSubsystem, mShooterSubsystem, mHopperSubsystem);
 //}
    
  

   
}

  


