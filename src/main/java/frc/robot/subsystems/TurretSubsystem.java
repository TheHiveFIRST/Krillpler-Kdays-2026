package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.configs.TurretConfig;
public class TurretSubsystem extends SubsystemBase{
    TalonFX mTurretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);
    double TurretMotorTarget = 0;
    public static boolean turretAtTarget = false;
    public static boolean turretEnabled = true;
    PositionVoltage turnPositionRequest = new PositionVoltage(0);
    VelocityVoltage manualVelocityRequest = new VelocityVoltage(0);
    public TurretSubsystem() {
    
        mTurretMotor.getConfigurator().apply(TurretConfig.trackHub);
        



    }

    public double getMotorTarget(Rotation2d Rotationtarget){
        double unsafeMotorTarget = (Rotationtarget.getRadians() / (2*(Math.PI))) * TurretConstants.MOTOR_ROTATIONS_PER_TURRET_ROTATION;
        double safeMotorTarget;
        if (unsafeMotorTarget > TurretConstants.TURRET_LOOP_POINT){
            safeMotorTarget = unsafeMotorTarget - TurretConstants.MOTOR_ROTATIONS_PER_TURRET_ROTATION;
        } else {
            safeMotorTarget = unsafeMotorTarget;
        }
        return safeMotorTarget;
    }
    
    public void setTurnSetpoint(double position, AngularVelocity velocity) {
        mTurretMotor.setControl(turnPositionRequest.withPosition(position).withVelocity(velocity));
    }

    public void setTurretTarget(Rotation2d TurretTarget){
        setTurnSetpoint(getMotorTarget(TurretTarget), Units.RotationsPerSecond.of(0));
        TurretMotorTarget = getMotorTarget(TurretTarget);

    }

    public boolean isTurretAtTarget() {
        return (Math.abs(mTurretMotor.getPosition().getValueAsDouble() - TurretMotorTarget) < 0.05);
    }
    
    //Manual turret controls
    public void runTurretClockwise() {
        mTurretMotor.setControl(manualVelocityRequest.withVelocity(5));
    }

    public void runTurretCounterClockwise() {
        mTurretMotor.setControl(manualVelocityRequest.withVelocity(-5));
    }

    public void stopTurret() {
        mTurretMotor.stopMotor();
    }

    public void periodic() {
        turretAtTarget = isTurretAtTarget();
    }
    //commands
    public Command stopTurretCommand() {
         return run(
        () -> {
            stopTurret();
              });
    }

    public Command runTurretClockwiseCommand() {
         return run(
        () -> {
            runTurretClockwise();
              });
    }

    public Command runTurretCounterClockwiseCommand() {
         return run(
        () -> {
            runTurretCounterClockwise();
              });
    }

    public Command runTurretFerryPositionCommand(){
         return run(
            () -> {
                setTurretTarget(DriveSubsystem.robotYaw); //TODO: fix this. this is probably inverted or smth idk

            });
    }

    public Command runTurretHubCommand(){
         return run(
            () -> {
                setTurretTarget(DriveSubsystem.robotYaw); 
                //TODO: pass in target from calculations file instead of robot yaw once that is coded and these branches are merged

            });
    }



}
