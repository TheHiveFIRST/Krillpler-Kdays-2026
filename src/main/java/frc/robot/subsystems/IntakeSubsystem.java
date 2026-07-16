package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.configs.IntakeConfig;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax mIntakeMotor;
    
    //constructor
    public IntakeSubsystem() {
      mIntakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
      mIntakeMotor.configure(IntakeConfig.IntakerConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //methods
    public void setIntakePower(double power) {
      mIntakeMotor.set(power);
    }  
    
    public void stopIntake() {
      mIntakeMotor.stopMotor();
    }

    //Commands 
    public Command runIntakeCommand() {
         return run(
        () -> {
          setIntakePower(IntakeConstants.INTAKING_POWER);
              });
    }

    public Command stopIntakeCommand() {
         return run(
        () -> {
            stopIntake();
              });
    }

    public Command reverseIntakeCommand() {
         return run(
        () -> {
            setIntakePower(-IntakeConstants.INTAKING_REVERSE_POWER);
              });
    }
}