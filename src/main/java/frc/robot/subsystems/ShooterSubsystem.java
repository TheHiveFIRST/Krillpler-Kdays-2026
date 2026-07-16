package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import frc.robot.Constants.ShooterConstants;
import frc.robot.configs.ShootConfig.ShooterConfig;

import frc.robot.subsystems.DriveSubsystem;

import static edu.wpi.first.units.Units.*;

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
            runShooterPower(0.7 + ShooterPowerOffset);
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
    return new InstantCommand(() -> changeShootingPowerOffset(ShooterConstants.RPMOFFSET_INCREMENT));
    }
    
    public Command decreaseShootingRPMOffsetCommand(){
      return new InstantCommand(() -> changeShootingPowerOffset(-ShooterConstants.RPMOFFSET_INCREMENT));
    }

    @Override
    public void periodic() {
    }
}