package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class shootUnjamSequence extends SequentialCommandGroup{
    public shootUnjamSequence(HopperSubsystem mHopperSubsystem, IntakeSubsystem mIntakeSubsystem) {
    addCommands( 
      
      new ParallelCommandGroup(
        mHopperSubsystem.runShootCommand().withTimeout(0.5),
        mIntakeSubsystem.runIntakeCommand().withTimeout(0.5)
      ),
      new WaitCommand(0.01),
      new ParallelCommandGroup(
        mHopperSubsystem.runShootUnjamCommand().withTimeout(0.2),
        mIntakeSubsystem.revereIntakeCommand().withTimeout(0.2)
      ),
      
      new WaitCommand(0.01));       
      
  }
}
