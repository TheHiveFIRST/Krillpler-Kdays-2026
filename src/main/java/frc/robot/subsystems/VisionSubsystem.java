package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {
    public final PhotonCamera photonCamera1;
    
    public VisionSubsystem() {
        photonCamera1 = new PhotonCamera("camera1");
    }

    public boolean hasResults() {
        // Query the latest result from PhotonVision
        var result = photonCamera1.getLatestResult();
        boolean hasTargets = result.hasTargets();
        return hasTargets;
    }
    public PhotonTrackedTarget getCurrentBestResults() {
        // Query the latest result from PhotonVision
        var result = photonCamera1.getLatestResult();
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        return bestTarget;
    }

    public PhotonTrackedTarget getCurrentIDResults(int ID) {
        // Query the latest result from PhotonVision
        var result = photonCamera1.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget targetWithID = null;
        for (PhotonTrackedTarget target : targets) {
            int aprilTagID = target.getFiducialId();
            if (aprilTagID == ID) {
                targetWithID = target;
            }
        }
        return targetWithID;
    }

    public double getTargetYaw(PhotonTrackedTarget target) {
        return target.getYaw();
    }

    public double getTargetArea(PhotonTrackedTarget target) {
        return target.getArea();
    }

    public double getTargetPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

}//
    
