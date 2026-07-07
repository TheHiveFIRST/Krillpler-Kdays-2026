package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  // declare motors/controllers here
    private final WPI_TalonSRX mIntakeMotorLeader;
    private final WPI_TalonSRX mIntakeMotorFollower;
    
    public static boolean isIntaking = false;
    
    public IntakeSubsystem() {
      mIntakeMotorLeader = new WPI_TalonSRX(IntakeConstants.INTAKE_LEADER_ID);
      mIntakeMotorFollower = new WPI_TalonSRX(IntakeConstants.INTAKE_FOLLOWER_ID);

      TalonSRXConfiguration config = new TalonSRXConfiguration();
      config.peakCurrentLimit = 40; //amps
      config.peakCurrentDuration = 200;
      config.continuousCurrentLimit = 30;
      mIntakeMotorLeader.configAllSettings(config);
      mIntakeMotorFollower.configAllSettings(config);
      mIntakeMotorLeader.enableCurrentLimit(true);
      mIntakeMotorFollower.enableCurrentLimit(true);
      mIntakeMotorFollower.setInverted(true);

      
    }

    public void setIntakePower(double speed) {
      mIntakeMotorLeader.set(speed);
      mIntakeMotorFollower.set(speed);
    }
    

    

    public void stopIntake() {
      mIntakeMotorLeader.stopMotor();
      mIntakeMotorFollower.stopMotor();
    }

    

    //Commands 
    
    

    public Command runIntakeCommand() {
         return run(
        () -> {
          if (HopperSubsystem.isShooting) {
            setIntakePower(IntakeConstants.INTAKING_PARTIAL_POWER);
          } else {
          setIntakePower(IntakeConstants.INTAKING_FULL_POWER);
          }
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

    public Command deployIntakeCommand() {
         return run(
        () -> {
            
              });
    }



    
}