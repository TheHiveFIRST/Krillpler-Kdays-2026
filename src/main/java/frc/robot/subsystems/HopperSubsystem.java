package frc.robot.subsystems; // Marshal lmk if this is fine.

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.configs.HopperConfig;

public class HopperSubsystem extends SubsystemBase{

    private final WPI_TalonSRX mTopRoller;
    private final WPI_TalonSRX mTopBelt;
    private final WPI_TalonSRX mIndexer;
    private final WPI_TalonSRX mBottomBelt;
    
    public boolean isShooting = false; // Just to track if we are shooting or not just in case. 
    // Currently not in use.
     

    public HopperSubsystem(){
        
        mTopRoller = new WPI_TalonSRX(HopperConstants.TOP_ROLLERS_ID);
        mBottomBelt = new WPI_TalonSRX(HopperConstants.BOTTOM_BELTS_ID);
        mTopBelt = new WPI_TalonSRX(HopperConstants.TOP_BELTS_ID);
        mIndexer = new WPI_TalonSRX(HopperConstants.INDEXER_ID);

        HopperConfig.configure(mTopRoller);
        HopperConfig.configure(mBottomBelt);
        HopperConfig.configure(mTopBelt);
        HopperConfig.configure(mIndexer);
    } 

    public void runTopRoller(double speed) {
        mTopRoller.set(speed);
    }

    public void runShoot(double speed) {
        mTopBelt.set(speed);
        mBottomBelt.set(speed);
        mIndexer.set(speed);
    }

    public void stopIntake() {
        runTopRoller(0);
        runShoot(0);
    }

    public Command runIntakeCommand() { // Moodshal said this can have problemss. 
        return run(
        () -> {
            runTopRoller(HopperConstants.HOPPER_SPEED);
              });
    }

    public Command runShootCommand() {
        return run(
            () -> {
                runShoot(HopperConstants.HOPPER_SPEED);
            });
    }

    public Command stopIntakeCommand() {
        return run(
            () -> {
                stopIntake();
            });
    }

    @Override
    public void periodic() {
    }

}
