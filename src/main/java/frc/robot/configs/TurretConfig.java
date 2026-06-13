package frc.robot.configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.TurretConstants;

public final class TurretConfig {
    public static final class TurretConfigs {
        public static final TalonFXConfiguration trackHub = new TalonFXConfiguration();

        static {

            trackHub:
                trackHub.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                trackHub.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                trackHub.CurrentLimits.SupplyCurrentLimit = 30;
                trackHub.CurrentLimits.SupplyCurrentLimitEnable = true;

                trackHub.Slot0.kS = 0;
                trackHub.Slot0.kV = 0;
                trackHub.Slot0.kA = 0;
                trackHub.Slot0.kP = TurretConstants.kP;

                trackHub.MotionMagic.MotionMagicAcceleration = 0;
                trackHub.MotionMagic.MotionMagicCruiseVelocity = 0;
        }
    }
}