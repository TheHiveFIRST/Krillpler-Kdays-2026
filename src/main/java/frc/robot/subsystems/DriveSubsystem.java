// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import static edu.wpi.first.units.Units.Degrees;

public class DriveSubsystem extends SubsystemBase {
  //create 4 MAXSwerveModules 
  
  private final MaxSwerveModule mFrontLeft = new MaxSwerveModule(
    DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
    DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
    DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);
  
  private final MaxSwerveModule mFrontRight = new MaxSwerveModule(
    DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
    DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
    DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);
  
  private final MaxSwerveModule mBackLeft = new MaxSwerveModule(
    DriveConstants.BACK_LEFT_DRIVING_CAN_ID,
    DriveConstants.BACK_LEFT_TURNING_CAN_ID,
    DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MaxSwerveModule mBackRight = new MaxSwerveModule(
    DriveConstants.BACK_RIGHT_DRIVING_CAN_ID,
    DriveConstants.BACK_RIGHT_TURNING_CAN_ID,
    DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  //mGyro sensor/IMU (usb input type to roborio)
  private final AHRS mGyro = new AHRS(NavXComType.kUSB1); 
  public static boolean useInvertedGyro = true;
  
  private final Field2d field2d = new Field2d();

  public static double hubDistance = 0; 

  
  public double targetx = 0;
  public double targety = 0;
  public double targetangle = 0;

  public static double gyrooffset = 0; 
  
  // constants for velocity & rotation logging
  private double prevLinearVel            = 0;
  private double prevOmega                = 0;
  private double prevCharTime             = 0;
 
  private double peakLinearVelocity       = 0;
  private double peakLinearAcceleration   = 0;
  private double peakAngularVelocity      = 0;
  private double peakAngularAcceleration  = 0;
 
  public Rotation2d desiredAngle;
  
  private final SwerveDrivePoseEstimator mPoseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.DriveKinematics,
          getGyroRotation(),
          new SwerveModulePosition[] {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mBackLeft.getPosition(),
            mBackRight.getPosition()
          },
          new Pose2d());
          //VecBuilder.fill(DriveConstants.POSE_ESTIMATOR_N1,DriveConstants.POSE_ESTIMATOR_N2, Units.degreesToRadians(5)),
          //VecBuilder.fill(DriveConstants.POSE_ESTIMATOR_2_N1, DriveConstants.POSE_ESTIMATOR_2_N1, Units.degreesToRadians(30)));

  //Odometry class for tracking robot pose 
  SwerveDriveOdometry Odometry = new SwerveDriveOdometry(
    DriveConstants.DriveKinematics,
    getGyroRotation(), //inversion as NavX is CCW+
    new SwerveModulePosition[] {
        mFrontLeft.getPosition(),
        mFrontRight.getPosition(),
        mBackLeft.getPosition(),
        mBackRight.getPosition()
  });

  
  public DriveSubsystem() {
    //usage reporting for MAXSwerve template 
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = null;
    }
    AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(15, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(12, 0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );   
      
  }  

  
  @Override
  public void periodic(){
  //updates Odometry in periodic block 
    Odometry.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mBackLeft.getPosition(),
            mBackRight.getPosition()
        });

    updateVisionOdometry();

    //adding field map to smart dashboard 
    field2d.setRobotPose(mPoseEstimator.getEstimatedPosition());
    SmartDashboard.putData(field2d);

    hubDistance = getHubDistance();

    SmartDashboard.putNumber("Driving/hub distance", getHubDistance());
    SmartDashboard.putNumber("Position", mBackRight.getPosition().angle.getRadians());
    SmartDashboard.putNumber("Driving/gyro", -mGyro.getAngle());
    SmartDashboard.putNumber("Driving/newgyro", mPoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Driving/heading", getHeading());
    SmartDashboard.putNumber("Driving/Pose X", getVisionPose().getX());
    SmartDashboard.putNumber("Driving/Pose Y", getVisionPose().getY());
    SmartDashboard.putString("Driving/Alliance", Constants.getCurrentAlliance().toString());
    SmartDashboard.putNumber("Driving/HubPoseX", DriveConstants.getHubPose().getX());
    SmartDashboard.putNumber("Driving/HubPoseY", DriveConstants.getHubPose().getY());
    SmartDashboard.putNumber("Driving/kp", DriveConstants.ROTATION_KP);
    SmartDashboard.putNumber("ODOM X", Odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("VISION X", mPoseEstimator.getEstimatedPosition().getX());
SmartDashboard.putNumber("Driving/x", targetx);
    SmartDashboard.putNumber("Driving/y", targety);
    SmartDashboard.putNumber("Driving/angle", targetangle*180/Math.PI);

    
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  public Pose2d getPose() {
    return mPoseEstimator.getEstimatedPosition();
}

  /**
   * Resets the Odometry to the specified pose.
   * @param pose The pose to which to set the Odometry.
   */
  public void resetPose(Pose2d pose) {
    mPoseEstimator.resetPosition(
    getGyroRotation(),
    new SwerveModulePosition[] {
        mFrontLeft.getPosition(),
        mFrontRight.getPosition(),
        mBackLeft.getPosition(),
        mBackRight.getPosition()
    },
    pose
);
  }
  /**
   * Returns the currentlyVISION estimated pose of the robot.
   * @return The pose.
   */
  public Pose2d getVisionPose() {
    return mPoseEstimator.getEstimatedPosition();
  }
  public Rotation2d getRotationPose2d() {
    return mPoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Resets the Odometry to the specified pose.
   * @param pose The pose to which to set the Odometry.
   */
  public void resetPoseEstimator(Pose2d pose) {
    mPoseEstimator.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mBackLeft.getPosition(),
            mBackRight.getPosition()
        },
        pose);
  }


  public void driveJoystick(double xJoystick, double yJoystick, double rotJoystick, boolean fieldRelative) {
    
    //convert joystick input (-1, 1) to m/s for drivetrain 
    double xSpeedDelivered = xJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND; 
    double ySpeedDelivered = yJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotDelivered = rotJoystick * DriveConstants.MAX_ANGULAR_SPEED;

    driveChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
  }

  public void driveIntakeAlign(double xJoystick, double yJoystick, boolean fieldRelative) {

   double xSpeed = xJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
   double ySpeed = yJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND;

   double rotSpeed = 0;

   if (Math.abs(xSpeed) > 0.05 || Math.abs(ySpeed) > 0.05) {
      double desiredAngle = Math.atan2(ySpeed, xSpeed); // radians
     double currentAngle = getGyroRotation().getRadians();
     double angleError = MathUtil.angleModulus(desiredAngle - currentAngle);
     //TODO: TEST AND TUNE THE PID
     rotSpeed = angleError * DriveConstants.INTAKE_ALIGN_KP;
    }

  driveChassisSpeeds(xSpeed, ySpeed, rotSpeed, fieldRelative);
  }

  public void driveDiagonalBumpAlign(double xJoystick, double yJoystick, boolean fieldRelative, double targetDegrees) {

    double xSpeed = xJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double ySpeed = yJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND;

    double desiredAngle = Math.toRadians(targetDegrees);
    double currentAngle = getGyroRotation().getRadians();
    double angleError = MathUtil.angleModulus(desiredAngle - currentAngle);

    double rotSpeed = angleError * DriveConstants.DIAGONAL_ALIGN_kP;

  driveChassisSpeeds(xSpeed, ySpeed, rotSpeed, fieldRelative);
  }



  public void driveChassisSpeeds(double xSpeed, double ySpeed, double rotValue, boolean fieldRelative){
    // clamps speed to be within max/min range 
    double xSpeedClamped = MathUtil.clamp(xSpeed, -DriveConstants.MAX_SPEED_METERS_PER_SECOND,DriveConstants.MAX_SPEED_METERS_PER_SECOND); 
    double ySpeedClamped = MathUtil.clamp(ySpeed, -DriveConstants.MAX_SPEED_METERS_PER_SECOND,DriveConstants.MAX_SPEED_METERS_PER_SECOND); 
    double rotDelivered = MathUtil.clamp(rotValue, -DriveConstants.MAX_ANGULAR_SPEED, DriveConstants.MAX_ANGULAR_SPEED);

    //convert chassis speed to swerve module states (motor output); field relative or robot relative 
    var swerveModuleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(
      fieldRelative 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedClamped, ySpeedClamped, 
          rotDelivered, getGyroRotation())
        
        : new ChassisSpeeds(xSpeedClamped, ySpeedClamped, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    mFrontLeft.setDesiredState(swerveModuleStates[0]);
    mFrontRight.setDesiredState(swerveModuleStates[1]);
    mBackLeft.setDesiredState(swerveModuleStates[2]);
    mBackRight.setDesiredState(swerveModuleStates[3]);

  }
 
  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    mFrontLeft.setDesiredState(swerveModuleStates[0]);
    mFrontRight.setDesiredState(swerveModuleStates[1]);
    mBackLeft.setDesiredState(swerveModuleStates[2]);
    mBackRight.setDesiredState(swerveModuleStates[3]);
  }  

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.DriveKinematics.toChassisSpeeds(
        mFrontLeft.getState(),
        mFrontRight.getState(),
        mBackLeft.getState(),
        mBackRight.getState()
        );
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    mFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    mFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    mFrontLeft.setDesiredState(desiredStates[0]);
    mFrontRight.setDesiredState(desiredStates[1]);
    mBackLeft.setDesiredState(desiredStates[2]);
    mBackRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    mFrontLeft.resetEncoders();
    mFrontRight.resetEncoders();
    mBackLeft.resetEncoders();
    mBackRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mGyro.reset();
  }

  public Rotation2d getGyroRotation(){
    double angle = mGyro.getAngle(); 
    return Rotation2d.fromDegrees(-angle
      /*useInvertedGyro ? -angle : angle*/
    );
  }
  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation().getDegrees();
  }


  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return mGyro.getRate() * (useInvertedGyro ? -1.0 : 1.0);
  }

    /** Updates the field relative position of the robot. */
  public void updateVisionOdometry() {
    mPoseEstimator.update(
          getGyroRotation(),
        new SwerveModulePosition[] {
          mFrontLeft.getPosition(),
          mFrontRight.getPosition(),
          mBackLeft.getPosition(),
          mBackRight.getPosition()
        });

    boolean doRejectUpdate = false;

    Optional<EstimatedRobotPose> visionEstimator = VisionSubsystem.visionEst;
    
    if(Math.abs(mGyro.getRate()) > 720) {// if our angular velocity is greater than 720 degrees per second, ignore vision updates
      doRejectUpdate = true;
    }
    if(visionEstimator.isPresent() && !doRejectUpdate) {
      EstimatedRobotPose estVision = visionEstimator.get();

      if (estVision.targetsUsed.size() > 1) {
        mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
        DriveConstants.VISION_STD_PTG2_N1, 
        DriveConstants.VISION_STD_PTG2_N2, 
        Units.degreesToRadians(5)));
      }
      else {
        mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
        DriveConstants.VISION_STD_PTG1_N1, 
        DriveConstants.VISION_STD_PTG1_N2, 
        Units.degreesToRadians(5)));
      }

      mPoseEstimator.addVisionMeasurement(
        estVision.estimatedPose.toPose2d(),
        estVision.timestampSeconds
      );
    }
  }
  
  public double getFerryDistance() {
      return getShotDistance(DriveConstants.getFerryPose(getVisionPose().getTranslation()).toPose2d().getTranslation());
  }

  public double getHubDistance() {
        return getShotDistance(DriveConstants.getHubPose().toPose2d().getTranslation());
    }


  public double getShotDistance(Translation2d targetPose) {
        Pose2d drivePose = getVisionPose();
        double centerToTargetMeters = drivePose.getTranslation().getDistance(targetPose);
        // double centerToShooterMeters = DriveConstants.shooterSideOffset;
        // double shooterToTargetMeters = Math.sqrt(Math.pow(centerToTargetMeters, 2.0) - Math.pow(centerToShooterMeters, 2.0));
        return centerToTargetMeters;
  }

  
  
  public void incrementKP(){ DriveConstants.ROTATION_KP += DriveConstants.KP_INCREMENT;};

  public void decrementKP(){ DriveConstants.ROTATION_KP -= DriveConstants.KP_INCREMENT;};


  //command to set module positions to an X shape for defense 
  public Command defensePosition(){
    return run(
      () -> {
        setX();
      });
  }

  //command to reset gyro
  public Command resetGyro(){
    return run(
      () -> {
        zeroHeading();
      });
  }
  public Command characterizeLinear(double durationSeconds) {
    return Commands.sequence(
        Commands.runOnce(() -> {
            peakLinearVelocity     = 0;
            peakLinearAcceleration = 0;
            prevLinearVel          = 0;
            prevCharTime           = Timer.getFPGATimestamp();
            resetEncoders();
            System.out.println("[Char] Linear test started.");
        }),
        Commands.run(() -> {
            driveRobotRelative(new ChassisSpeeds(DriveConstants.MAX_SPEED_METERS_PER_SECOND, 0, 0));
            logLinearChar();
        }, this).withTimeout(durationSeconds),
        Commands.runOnce(() -> {
            driveRobotRelative(new ChassisSpeeds());
            System.out.printf(
                "[Char] Linear done.  Peak vel=%.3f m/s   Peak accel=%.3f m/s^2%n",
                peakLinearVelocity, peakLinearAcceleration);
        })
    );
}
 
/**
 * Spins the robot in place at full rotational speed to find:
 *   - Max angular velocity     -> "Char/Peak Angular Velocity (rad/s)"
 *   - Max angular acceleration -> "Char/Peak Angular Accel (rad/s^2)"
 *
 * Robot spins in place -- 2 s is usually enough.
 * Run 2-3 times and use a value slightly below the measured peak.
 *
 * @param durationSeconds How long to run (2.0 s recommended)
 */
public Command characterizeAngular(double durationSeconds) {
    return Commands.sequence(
        Commands.runOnce(() -> {
            peakAngularVelocity     = 0;
            peakAngularAcceleration = 0;
            prevOmega               = 0;
            prevCharTime            = Timer.getFPGATimestamp();
            mGyro.reset();
            System.out.println("[Char] Angular test started.");
        }),
        Commands.run(() -> {
            driveRobotRelative(new ChassisSpeeds(0, 0, DriveConstants.MAX_ANGULAR_SPEED));
            logAngularChar();
        }, this).withTimeout(durationSeconds),
        Commands.runOnce(() -> {
            driveRobotRelative(new ChassisSpeeds());
            System.out.printf(
                "[Char] Angular done.  Peak omega=%.3f rad/s   Peak alpha=%.3f rad/s^2%n",
                peakAngularVelocity, peakAngularAcceleration);
        })
    );
}
 
private void logLinearChar() {
    double now = Timer.getFPGATimestamp();
    double dt  = now - prevCharTime;
    if (dt <= 0) return;
 
    // Average speed magnitude across all four modules
    double linearVel = (
        Math.abs(mFrontLeft.getState().speedMetersPerSecond)  +
        Math.abs(mFrontRight.getState().speedMetersPerSecond) +
        Math.abs(mBackLeft.getState().speedMetersPerSecond)   +
        Math.abs(mBackRight.getState().speedMetersPerSecond)
    ) / 4.0;
 
    double linearAccel = (linearVel - prevLinearVel) / dt;
 
    peakLinearVelocity     = Math.max(peakLinearVelocity,     linearVel);
    peakLinearAcceleration = Math.max(peakLinearAcceleration, Math.abs(linearAccel));
 
    SmartDashboard.putNumber("Char/Linear Velocity (m/s)",      linearVel);
    SmartDashboard.putNumber("Char/Linear Accel (m/s^2)",       linearAccel);
    SmartDashboard.putNumber("Char/Peak Linear Velocity (m/s)", peakLinearVelocity);
    SmartDashboard.putNumber("Char/Peak Linear Accel (m/s^2)",  peakLinearAcceleration);
 
    prevLinearVel = linearVel;
    prevCharTime  = now;
}
 
private void logAngularChar() {
    double now = Timer.getFPGATimestamp();
    double dt  = now - prevCharTime;
    if (dt <= 0) return;
 
    // Derived from wheel speeds so it agrees with your kinematics model
    double omega = getRobotRelativeSpeeds().omegaRadiansPerSecond;
    double alpha = (omega - prevOmega) / dt;
 
    peakAngularVelocity     = Math.max(peakAngularVelocity,     Math.abs(omega));
    peakAngularAcceleration = Math.max(peakAngularAcceleration, Math.abs(alpha));
 
    SmartDashboard.putNumber("Char/Angular Velocity (rad/s)",      omega);
    SmartDashboard.putNumber("Char/Angular Accel (rad/s^2)",       alpha);
    SmartDashboard.putNumber("Char/Peak Angular Velocity (rad/s)", peakAngularVelocity);
    SmartDashboard.putNumber("Char/Peak Angular Accel (rad/s^2)",  peakAngularAcceleration);
    // NavX cross-check -- should be close to the kinematics value above;
    // a big mismatch usually means wheel slip
    SmartDashboard.putNumber("Char/NavX Rate (rad/s)",             Math.toRadians(mGyro.getRate()));
 
    prevOmega    = omega;
    prevCharTime = now;
}
 


}

  

