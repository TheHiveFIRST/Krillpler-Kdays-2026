package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

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
                //translation joystick inputs are cubed, allowing for precise movement
                MathUtil.applyDeadband((-mDriverController.getLeftY() * -mDriverController.getLeftY() * -mDriverController.getLeftY())*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband((-mDriverController.getLeftX() * -mDriverController.getLeftX() * -mDriverController.getLeftX())*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                true);},
            mDriveSubsystem));

       
    //krillpler is lazy and doesnt do anything unless you tell it to
    mIntakeSubsystem.setDefaultCommand(mIntakeSubsystem.stopIntakeCommand());
    mhoppersubsystem.setDefaultCommand(mhoppersubsystem.stopShootingCommand());
    mShooterSubsystem.setDefaultCommand(mShooterSubsystem.stopShooterCommand());
        
        
        
     }

    
    private void configureBindings() {
      
        //OPERATOR CONTROLS
       
        mOperatorController.leftBumper().whileTrue(mShooterSubsystem.runShooterCommand());
        //POV down is used for unjamming
        mOperatorController.povDown().whileTrue(mIntakeSubsystem.reverseIntakeCommand());
        mOperatorController.povDown().whileTrue(mhoppersubsystem.reverseCommand());

        


        //DRIVER CONTROLS
        
        
        mDriverController.leftBumper().whileTrue(mIntakeSubsystem.runIntakeCommand());
        
        //Shooter will automatically turn on when the shoot button is pressed, and considering
        //krillpler's BPS, there isnt much need to spin up beforehand
        mDriverController.rightBumper().whileTrue(RunHopperUnjam().repeatedly());
        mDriverController.rightBumper().whileTrue(mShooterSubsystem.runShooterCommand());
        
        //for passing/unjamming/human player feeding
        mDriverController.povUp().whileTrue(mIntakeSubsystem.reverseIntakeCommand());
       
        //driving utilities
        mDriverController.x().whileTrue(mDriveSubsystem.defensePosition());
        mDriverController.start().whileTrue(mDriveSubsystem.resetGyro()); 
        mDriverController.rightTrigger(0.3).onChange(toggleSlowMode());

    }

    public void zeroGyroHeading(){
      mDriveSubsystem.zeroHeading();
    }

    

    
    
    public Command toggleSlowMode(){
        return new InstantCommand(() -> slowMode = !slowMode);
    }

    
    public Command RunHopperUnjam(){
        return new shootUnjamSequence(mhoppersubsystem, mIntakeSubsystem);
    }
 
}

  


