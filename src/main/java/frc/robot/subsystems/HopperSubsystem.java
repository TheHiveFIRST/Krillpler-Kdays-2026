package frc.robot.subsystems; // Marshal lmk if this is fine.

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.configs.HopperConfig;
import frc.robot.configs.ShootConfig.ShooterConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class HopperSubsystem extends SubsystemBase{

    private final WPI_TalonSRX mTopRoller;
    private final WPI_TalonSRX mRollerFloor;
    private final WPI_TalonSRX mIndexer;
    private final SparkMax mBelts;
    
    public static boolean isShooting = false; // Just to track if we are shooting or not just in case. 
    // Currently not in use.
     

    public HopperSubsystem(){
        
        mTopRoller = new WPI_TalonSRX(HopperConstants.TOP_ROLLERS_ID);
        mRollerFloor = new WPI_TalonSRX(HopperConstants.ROLLER_FLOOR_ID);
        mBelts = new SparkMax(HopperConstants.TOP_BELTS_ID, MotorType.kBrushless);
        mIndexer = new WPI_TalonSRX(HopperConstants.INDEXER_ID);

        HopperConfig.configure(mTopRoller);
        mBelts.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        HopperConfig.configure(mIndexer);
        HopperConfig.configure(mRollerFloor);
    } 

    public void runTopRoller(double speed) {
        mTopRoller.set(speed);
    }

    public void runShoot(double speed) {
        mBelts.set(speed);
        mIndexer.set(-speed);
        mRollerFloor.set(speed);
        mTopRoller.set(0.5 * speed);
    }
    public void runShootUnjam(double speed) {
        mBelts.set(speed);
        mIndexer.set(-speed);
        mRollerFloor.set(-speed);
        mTopRoller.set(-speed);
    }

    public void stopShooting() {
        
        runShoot(0);
    }

    public Command runTopRollerCommand() { // Moodshal said this can have problemss. 
        return run(
        () -> {
            runTopRoller(HopperConstants.HOPPER_SLOW_SPEED);
              });
    }

    public Command runShootCommand() {
        return run(
            () -> {
                if (IntakeSubsystem.isIntaking){
                    runShoot(HopperConstants.HOPPER_SLOW_SPEED);
                } else {
                    runShoot(HopperConstants.HOPPER_FAST_SPEED);
                }
            });
    }
    public Command runShootUnjamCommand() {
        return run(
            () -> {
                if (IntakeSubsystem.isIntaking){
                    runShootUnjam(HopperConstants.HOPPER_SLOW_SPEED);
                } else {
                    runShootUnjam(HopperConstants.HOPPER_FAST_SPEED);
                }
            });
    }
    

    public Command stopShootingCommand() {
        return run(
            () -> {
                stopShooting();
            });
    }

    @Override
    public void periodic() {
    }

}

//hi