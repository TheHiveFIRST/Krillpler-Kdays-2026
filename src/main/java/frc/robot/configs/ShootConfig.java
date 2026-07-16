package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ShooterConstants;



public final class ShootConfig {
    public static final class ShooterConfig {
        public static final SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
        

        static {
            // Leader configuration
            shooterLeaderConfig
                   
                .smartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT)
                .idleMode(IdleMode.kCoast);
            
            shooterFollowerConfig
                
                .inverted(true) 
                .smartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT)
                .idleMode(IdleMode.kCoast);

            
        }
    }

    
    
}