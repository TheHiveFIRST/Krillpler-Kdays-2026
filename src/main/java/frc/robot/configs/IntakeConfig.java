package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.IntakeConstants;

public class IntakeConfig {
    public static final class IntakerConfig {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
        
        

        static {
            // Leader configuration
            intakeConfig
                .smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT)
                .idleMode(IdleMode.kCoast);
            // Follower configuration - set to follow Leader (ID 14) and invert
            

            
        }
    }
}
