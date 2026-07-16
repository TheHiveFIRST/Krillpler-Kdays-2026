package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


//un-comment this if you need smartDashbord to run diagnostics
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



import frc.robot.Constants.ShooterConstants;
import frc.robot.configs.ShootConfig.ShooterConfig;



public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax mShooterLeader; 
    private final SparkMax mShooterFollower;
    


    //initialized variables 
    private double ShooterPowerOffset = 0; 
    
    public ShooterSubsystem() {
        
        mShooterLeader = new SparkMax(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);
        mShooterFollower = new SparkMax(ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless);
    
        mShooterLeader.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterFollower.configure(ShooterConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    // Methods
    public void runShooterPower(double motorPower) {
        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);
    }

    private void changeShootingPowerOffset(double amount) {
        ShooterPowerOffset += amount;
    }

    //Commands to run the shooter
    
    public Command runShooterCommand() {
         return run(
        () -> {
            runShooterPower(ShooterConstants.SHOOTER_BASE_POWER + ShooterPowerOffset);
              });
    }

    public Command stopShooterCommand() {
         return run(
        () -> {
            runShooterPower(0);
              });
    }
    
    //Power adjustment
    
        public Command increaseShootingRPMOffsetCommand(){
    return new InstantCommand(() -> changeShootingPowerOffset(ShooterConstants.POWEROFFSET_INCREMENT));
    }
    
    public Command decreaseShootingRPMOffsetCommand(){
      return new InstantCommand(() -> changeShootingPowerOffset(-ShooterConstants.POWEROFFSET_INCREMENT));
    }

    @Override
    public void periodic() {
    }
}