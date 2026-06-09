package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
public class HopperSubsystem extends SubsystemBase{
    private final WPI_TalonSRX mToproller;
    private final WPI_TalonSRX mBelts;
    private final WPI_TalonSRX mKicker;

    
    
    public HopperSubsystem(){

    
    
        mToproller = new WPI_TalonSRX(HopperConstants.TOP_ROLLERS_ID);
        mBelts = new WPI_TalonSRX(HopperConstants.BELTS_ID);
        mKicker = new WPI_TalonSRX(HopperConstants.KICKER_ID);
    } 

}
