package frc.robot.configs;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.HopperConstants;

public final class HopperConfig {

    public static void configure(WPI_TalonSRX motor) {
        motor.configFactoryDefault();        
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configContinuousCurrentLimit(HopperConstants.HOPPER_CURRENT_LIMIT);
        motor.enableCurrentLimit(true);
    }
}
