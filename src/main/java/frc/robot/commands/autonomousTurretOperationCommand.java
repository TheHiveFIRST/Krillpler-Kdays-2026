package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.TurretSubsystem;

public class autonomousTurretOperationCommand extends RunCommand {

    public autonomousTurretOperationCommand(
            TurretSubsystem mTurretSubsystem,
            DriveSubsystem mDriveSubsystem) {

        super(
            () -> {
            
            if (mDriveSubsystem.inAliianceZone){
                mTurretSubsystem.targetHub();
            } else {
                mTurretSubsystem.setTurretTarget(mDriveSubsystem.getGyroRotation());

            }
            },
            mTurretSubsystem
        
        );
    }
}





