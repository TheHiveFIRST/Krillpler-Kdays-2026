package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class shootUnjamSequence extends SequentialCommandGroup{
    public shootUnjamSequence(HopperSubsystem mHopperSubsystem) {
    addCommands( 
      mHopperSubsystem.runShootCommand().withTimeout(0.6),
      new WaitCommand(0.01),
      mHopperSubsystem.runShootUnjamCommand().withTimeout(0.2),
      new WaitCommand(0.01));       
      
  }
}
