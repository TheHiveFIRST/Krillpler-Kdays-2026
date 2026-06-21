package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {
    public final PhotonCamera photonCamera1;
    public final AprilTagFieldLayout kTagLayout;
    public final Transform3d kRobotToCam;
    public final PhotonPoseEstimator photonEstimator;
    public static Optional<EstimatedRobotPose> visionEst;
    
    public VisionSubsystem() {
        photonCamera1 = new PhotonCamera("camera1");

        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        kRobotToCam = new Transform3d(new Translation3d(VisionConstants.robotToCameraX, VisionConstants.robotToCameraY, VisionConstants.robotToCameraZ), new Rotation3d(0, 0, 0));
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
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

    public Optional<EstimatedRobotPose> getVisionEst() {
        // Query the latest result from PhotonVision
        var result = photonCamera1.getLatestResult();
        Optional<EstimatedRobotPose> visionEstLocal = photonEstimator.estimateCoprocMultiTagPose(result);
        if (visionEstLocal.isEmpty()) {
            visionEstLocal = photonEstimator.estimateLowestAmbiguityPose(result);
        }
        return visionEstLocal;
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

    @Override
    public void periodic() {
        visionEst = getVisionEst();
    }
}//
    
