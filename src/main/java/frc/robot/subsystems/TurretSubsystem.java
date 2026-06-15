package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.configs.TurretConfig;
public class TurretSubsystem extends SubsystemBase{
    TalonFX mTurretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);
    
    public TurretSubsystem() {
    
        mTurretMotor.getConfigurator().apply(TurretConfig.trackHub);



    }

    public double getMotorTarget(Rotation2d Rotationtarget){
        double unsafeMotorTarget = (Rotationtarget.getRadians() / (2*(Math.PI))) * TurretConstants.MOTOR_ROTATIONS_PER_TURRET_ROTATION;
        double safeMotorTarget = Math.min(Math.max(unsafeMotorTarget, TurretConstants.TURRET_MIN_ROM) , TurretConstants.TURRET_MAX_ROM);
        return safeMotorTarget;
    }
    

}
