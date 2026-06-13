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

import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax mShooterLeader;
    private final SparkMax mShooterFollower;
    private final Servo hood = new Servo(1);
    private final RelativeEncoder mShooterLeaderEncoder;
    private final RelativeEncoder mShooterFollowerEncoder; 
    private final PIDController mShooterPID;
    private final SimpleMotorFeedforward tempFF;


    //initialized variables 
    private double mTargetRPM;
    private double ShooterRPMOffset = 0; 
    private boolean mShooterEnabled = false;
    public boolean turretTweaking = true;
    //TODO: when turret is functional, set this to false upon initialization
  

    //sysID 
 
    

   

    public ShooterSubsystem() {
    
        mShooterLeader = new SparkMax(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);
        mShooterFollower = new SparkMax(ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless);
        


        mShooterLeader.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterFollower.configure(ShooterConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterLeaderEncoder = mShooterLeader.getEncoder();
        mShooterFollowerEncoder = mShooterFollower.getEncoder();

        mShooterPID = new PIDController(ShooterConstants.LEADER_Kp, 
        ShooterConstants.LEADER_Ki, ShooterConstants.LEADER_Kd);
        
        tempFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS,
        ShooterConstants.LEADER_FF_kV, 
        ShooterConstants.LEADER_FF_kA);

        

    }

    // Methods
    public void runShooterPower(double motorPower) {
        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);
    }
    public double getAverageVelocity() {
        double sum = mShooterLeaderEncoder.getVelocity() + mShooterFollowerEncoder.getVelocity();
        double average = sum / 2;
        return average;
    }

    public void setShooterSpeeds(double setRPM, double RPMOffset) {
        double mCurrentRPM = getAverageVelocity();
        double pidOutput = mShooterPID.calculate(mCurrentRPM, setRPM + RPMOffset);
        double ffOutput = tempFF.calculate(setRPM + RPMOffset);
        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);
        runShooterPower(motorPower);
    }
    public double getShooterPIDF(double setRPM, double RPMOffset) {
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        double pidOutput = mShooterPID.calculate(mCurrentRPM, setRPM + RPMOffset);
        double ffOutput = tempFF.calculate(setRPM + RPMOffset);
        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);
        return motorPower;
    }

    public void hoodAim(Rotation2d angle){
        hood.set((angle.getRadians() - ShooterConstants.MIN_HOOD_ANGLE.getRadians()) 
        * ShooterConstants.HOOD_ANGLE_TO_SERVO_MULTIPLIER );
    }

    public void updateRPMs(){
        if (turretTweaking) {
            mTargetRPM = ShooterConstants.HUB_RPM;
            hoodAim(ShooterConstants.HUB_RPM_ANGLE);

        } else {
            //TODO: when calculations file is completed, pass in those values here
        }
    }

    private void changeShootingRPMOffset(double amount) {
        ShooterRPMOffset += amount;
    }

    //Commands 
     public Command runShooterPIDFCommand() {
         return run(
        () -> {
            setShooterSpeeds(mTargetRPM, ShooterRPMOffset);
              });
    }
    
    //toggles
    public Command toggleShooterCommand() {
        return new InstantCommand(() -> mShooterEnabled = !mShooterEnabled);}
 
    public Command increaseShootingRPMOffsetCommand(){
    return new InstantCommand(() -> changeShootingRPMOffset(ShooterConstants.RPMOFFSET_INCREMENT));
    }
    public Command decreaseShootingRPMOffsetCommand(){
      return new InstantCommand(() -> changeShootingRPMOffset(-ShooterConstants.RPMOFFSET_INCREMENT));
    }

    



    //sysID tests 
    //TODO: add sysid tests

    @Override
    public void periodic() {
      updateRPMs();  
        
        

        if (mShooterEnabled) {
            setShooterSpeeds(mTargetRPM, ShooterRPMOffset);
        } else {
            runShooterPower(0);
        }

        SmartDashboard.putNumber("Shooter/Target RPM", mTargetRPM + ShooterRPMOffset);
        SmartDashboard.putNumber("Shooter/Actual RPM", mShooterLeaderEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/RPM Offset", ShooterRPMOffset);
        
    boolean atSpeed = Math.abs(mShooterLeaderEncoder.getVelocity() - (mTargetRPM + ShooterRPMOffset))
        < ShooterConstants.VELOCITY_TOLERANCE;
        SmartDashboard.putBoolean("Shooter/Shooter Ready", atSpeed);
        SmartDashboard.putBoolean("Shooter/Shooter Toggled", mShooterEnabled);
        
    }



    // Tuning  
    // 
    //public enum TuningMode {
    //     KP, KV, KD, 
    // }
    // private TuningMode mCurrentTuningMode = TuningMode.KV;
    // // Cycle through tuning modes
    // public void cycleTuningMode() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             mCurrentTuningMode = TuningMode.KP;
    //             break;
    //         case KV:
    //             mCurrentTuningMode = TuningMode.KV;
    //             break;
    //         case KD:
    //             mCurrentTuningMode = TuningMode.KD;
    //             break;
    //     }
    // }
    //     public void incrementCurrentGain() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             incrementKP();
    //             break;
    //         case KV:
    //             incrementKV();
    //             break;
    //         case KD:
    //             incrementKD();
    //             break;
    //     }
    // }
    //     public void decrementCurrentGain() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             decrementKP();
    //             break;
    //         case KV:
    //             decrementKV();
    //             break;
    //         case KD:
    //             decrementKD();
    //             break;
    //     }
    // }

    // public void incrementKD() { mCurrentKD += ShooterConstants.KD_INCREMENT; } 
    // public void decrementKD() { mCurrentKD -= ShooterConstants.KD_INCREMENT; }

    // public void incrementKI() { mCurrentKI += ShooterConstants.KI_INCREMENT; } 
    // public void decrementKI() { mCurrentKI -= ShooterConstants.KI_INCREMENT; }

    // public void incrementKP() { mCurrentKP += ShooterConstants.KP_INCREMENT; } 
    // public void decrementKP() { mCurrentKP -= ShooterConstants.KP_INCREMENT; }

    // public void incrementKV() { mCurrentKV += ShooterConstants.KV_INCREMENT; } 
    // public void decrementKV() { mCurrentKV -= ShooterConstants.KV_INCREMENT; }
}