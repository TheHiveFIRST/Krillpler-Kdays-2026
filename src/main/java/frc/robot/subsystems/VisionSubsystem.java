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
    public final PhotonCamera photonCamera1; // PhotonVision camera object 
    public final AprilTagFieldLayout kTagLayout; // AprilTag field layout containing all tag locations on the field
    public final Transform3d kRobotToCam; // Transform from robot center to camera position
    public final PhotonPoseEstimator photonEstimator; // Estimates robot pose using detected AprilTags
    public static Optional<EstimatedRobotPose> visionEst; // Most recent vision pose estimate, updated every robot loop
    
    public VisionSubsystem() {
        photonCamera1 = new PhotonCamera("camera1"); // Connect to PhotonVision camera

        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // Load the official field's AprilTag locations
        
        // Camera position relative to robot center
        kRobotToCam = new Transform3d(new Translation3d(VisionConstants.robotToCameraX, VisionConstants.robotToCameraY, VisionConstants.robotToCameraZ), new Rotation3d(0, 0, 0));

        // Create pose estimator used to convert AprilTag detections into a field-relative robot pose
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    }

    /**
     * Returns whether the camera currently sees any valid targets.
     */
    public boolean hasResults() {
        // Query the latest result from PhotonVision
        var result = photonCamera1.getLatestResult();
        boolean hasTargets = result.hasTargets();
        return hasTargets;
    }

    /**
     * Returns the best target selected by PhotonVision.
     * Usually the closest/most reliable target.
     */
    public PhotonTrackedTarget getCurrentBestResults() {
        // Query the latest result from PhotonVision
        var result = photonCamera1.getLatestResult();
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        return bestTarget;
    }

    /**
     * Generates a robot pose estimate from AprilTag detections.
     *
     * Attempts a multi-tag pose solve first since it is typically
     * more accurate. If that fails, falls back to the lowest
     * ambiguity single-tag solution.
     */
    public Optional<EstimatedRobotPose> getVisionEst() {
        // Query the latest result from PhotonVision
        var result = photonCamera1.getLatestResult();
        Optional<EstimatedRobotPose> visionEstLocal = photonEstimator.estimateCoprocMultiTagPose(result);
        if (visionEstLocal.isEmpty()) {
            visionEstLocal = photonEstimator.estimateLowestAmbiguityPose(result);
        }
        return visionEstLocal;
    }

     /**
     * Searches all detected targets and returns the one matching
     * the requested AprilTag ID.
     *
     * Returns null if the tag is not currently visible.
     */
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

    /**
     * Returns horizontal offset from camera center.
     * Positive = right, Negative = left.
     */
    public double getTargetYaw(PhotonTrackedTarget target) {
        return target.getYaw();
    }

    /**
     * Returns target area as a percentage of the camera image.
     * Can be used as a rough distance estimate.
     */
    public double getTargetArea(PhotonTrackedTarget target) {
        return target.getArea();
    }

    /**
     * Returns vertical offset from camera center.
     * Positive = above center, Negative = below center.
     */
    public double getTargetPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    // DriveSubsystem reads VisionSubsystem.visionEst and uses it to update the SwerveDrivePoseEstimator.
    @Override
    public void periodic() {
        visionEst = getVisionEst(); // Update the latest vision pose estimate every robot loop
    }
}//
    
