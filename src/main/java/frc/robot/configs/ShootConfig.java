package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkFlexConfig;
//import frc.robot.Constants.IntakeConstants;

public final class ShootConfig {
    public static final class ShooterConfig {
        public static final SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
        

        static {
            // Leader configuration
            shooterLeaderConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kCoast);
            // Follower configuration - set to follow Leader (ID 14) and invert
            shooterFollowerConfig
                //.follow(ShooterConstants.SHOOTER_LEADER_CANID, true)
                .inverted(true)
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kCoast);

            
        }
    }

    
    
}