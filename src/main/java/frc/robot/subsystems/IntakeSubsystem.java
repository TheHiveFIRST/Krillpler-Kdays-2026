package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.configs.ShootConfig.IntakeConfigs;
// TODO: add necessary imports for cims/ talon SRX's

public class IntakeSubsystem extends SubsystemBase {
  // declare motors/controllers here
  
    
    


    public IntakeSubsystem() {
        
    }

    //Methods 
    public void stopIntake() {
           
        }

    public void runIntake(double speed){
            
        }


    //Commands 
    public Command runIntakeForwardCommand() {
         return run(
        () -> {
            runIntake(IntakeConstants.INTAKE_SPEED);
              });
    }

    

    public Command stopIntakeCommand() {
         return run(
        () -> {
            stopIntake();
              });
    }


    @Override
    public void periodic() {
    }
}