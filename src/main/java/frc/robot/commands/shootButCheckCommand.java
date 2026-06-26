package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.subsystems.TurretSubsystem;

import frc.robot.subsystems.HopperSubsystem;

public class shootButCheckCommand extends RunCommand {

    public shootButCheckCommand(
            TurretSubsystem mTurretSubsystem,
            ShooterSubsystem mShooterSubsystem,
            HopperSubsystem mHopperSubsystem) {

        super(
            () -> {
            if (mTurretSubsystem.isTurretAtTarget() && mShooterSubsystem.isAtSpeed()){
                mHopperSubsystem.runShootCommand();
            } else {
                mHopperSubsystem.stopShootingCommand();
            }
            },
            mTurretSubsystem
        
        );
    }
}





