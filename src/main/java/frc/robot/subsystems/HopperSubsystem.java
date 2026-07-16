package frc.robot.subsystems; // Marshal lmk if this is fine.

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.configs.HopperConfig;
import frc.robot.configs.ShootConfig.ShooterConfig;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class HopperSubsystem extends SubsystemBase{

    private final WPI_TalonSRX mTopRoller; //powers the top roller
    private final WPI_TalonSRX mRollerFloor; //powers the roller floor
    private final WPI_TalonSRX mIndexer; //Powers the 2 wheels that kick fuel up into the shooter
    private final SparkMax mBelts; // powers the beltdexer
    
    public static boolean isShooting = false; // Just to track if we are shooting or not just in case. 
    // Currently not in use.
     

    public HopperSubsystem(){
        //cims
        mTopRoller = new WPI_TalonSRX(HopperConstants.TOP_ROLLERS_ID);
        mRollerFloor = new WPI_TalonSRX(HopperConstants.ROLLER_FLOOR_ID);
        mIndexer = new WPI_TalonSRX(HopperConstants.INDEXER_ID);
        HopperConfig.configure(mTopRoller);
        HopperConfig.configure(mIndexer);
        HopperConfig.configure(mRollerFloor);

        //singular neo
        mBelts = new SparkMax(HopperConstants.TOP_BELTS_ID, MotorType.kBrushless);
        mBelts.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 

    

    public void runShoot(double speed) {
        mBelts.set(speed);
        mIndexer.set(-speed); //indexer motor is backwards
        mRollerFloor.set(speed);
        mTopRoller.set(speed);
    }

    //used for unjamming
    public void runHopper(double speed) { 
        mRollerFloor.set(speed);
        mTopRoller.set(speed);
    }

    //runs the top roller and roller floor slowly in reverse to release pressure on fuel in the beltdexer
    public void runShootUnjam(double speed) {
        mBelts.set(speed);
        mIndexer.set(-speed);
        mRollerFloor.set(-0.1 * speed);
        mTopRoller.set(-0.1 * speed);
    }

    public void stopShooting() {
        runShoot(0);
    }

    

    public Command runShootCommand() {
        return run(
            () -> {
                
                runShoot(HopperConstants.HOPPER_FAST_SPEED);
                
            });
    }
    public Command runShootUnjamCommand() {
        return run(
            () -> {
                
                runShootUnjam(HopperConstants.HOPPER_FAST_SPEED);
                
            });
    }

    

    public Command stopShootingCommand() {
        return run(
            () -> {
                stopShooting();
            });
    }
    public Command reverseCommand() {
        return run(
            () -> {
                runHopper(-HopperConstants.HOPPER_SLOW_SPEED);
            });
    }

    @Override
    public void periodic() {
    }

}

//hi