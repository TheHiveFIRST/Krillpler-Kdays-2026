package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.UnjamSequenceConstants;

public class shootUnjamSequence extends SequentialCommandGroup{
    public shootUnjamSequence(HopperSubsystem mHopperSubsystem, IntakeSubsystem mIntakeSubsystem) {
    addCommands( 
      
      new ParallelCommandGroup(
        mHopperSubsystem.runShootCommand().withTimeout(UnjamSequenceConstants.FORWARD_TIME),
        mIntakeSubsystem.runIntakeCommand().withTimeout(UnjamSequenceConstants.FORWARD_TIME)
      ),
      new WaitCommand(UnjamSequenceConstants.BUFFER_TIME),
      new ParallelCommandGroup(
        mHopperSubsystem.runShootUnjamCommand().withTimeout(UnjamSequenceConstants.BACKWARD_TIME),
        mIntakeSubsystem.reverseIntakeCommand().withTimeout(UnjamSequenceConstants.BACKWARD_TIME)
      ),
      
      new WaitCommand(UnjamSequenceConstants.BUFFER_TIME));       
      
  }
}
