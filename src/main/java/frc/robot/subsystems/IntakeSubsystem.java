package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.configs.IntakeConfig;
import frc.robot.configs.ShootConfig.ShooterConfig;


public class IntakeSubsystem extends SubsystemBase {
  // declare motors/controllers here
    private final SparkMax mIntakeMotor;
    private final RelativeEncoder mIntakeEncoder;
    
    public static boolean isIntaking = false;
    
    public IntakeSubsystem() {
       mIntakeMotor = new SparkMax(15, MotorType.kBrushless);
       mIntakeEncoder = mIntakeMotor.getEncoder();
        


        mIntakeMotor.configure(IntakeConfig.IntakerConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        
      
      
      

      
      

      
    }

    public void setIntakePower(double power) {
      mIntakeMotor.set(power);
      
    }

    public double getIntakeSpeed() {
      return mIntakeEncoder.getVelocity();
    }

    public void runIntakeSpeed(double speed){
      setIntakePower((speed - getIntakeSpeed()) * 0.01);
    }

    public void stopIntake() {
      mIntakeMotor.stopMotor();
      
    }

    

    //Commands 
    
    

    public Command runIntakeCommand() {
         return run(
        () -> {
          setIntakePower(1);
            isIntaking = true;
            
              });
    }

    public Command stopIntakeCommand() {
         return run(
        () -> {
            stopIntake();
            
            isIntaking = false;
              });
    }

    public Command revereIntakeCommand() {
         return run(
        () -> {
            setIntakePower(-0.5);
              });
    }



    
}