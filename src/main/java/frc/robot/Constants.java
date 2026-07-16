package frc.robot;

import java.util.Optional;


//hi
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static edu.wpi.first.units.Units.Degrees;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity (length, complexity).
 */
public final class Constants {
  public static Alliance getCurrentAlliance() {
    return DriverStation.getAlliance().get();
  }

  public static final class DriveConstants{ 
    //allowed max speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8; 
    public static final double MAX_ANGULAR_SPEED =  2 * Math.PI; // rad/s 
    //Chassis config - width, depth, CAN IDS and angular offset values in Designdoc.md
  
    public static final double WHEEL_CENTER_WIDTH = Units.inchesToMeters(12.75);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_CENTER_DEPTH = Units.inchesToMeters(10.25);
    // Distance between front and back wheels on robot
    //depth/frontback distance from robot center to each wheel 

    public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_CENTER_DEPTH, -WHEEL_CENTER_WIDTH),
        new Translation2d(WHEEL_CENTER_DEPTH, WHEEL_CENTER_WIDTH),
        new Translation2d(-WHEEL_CENTER_DEPTH, -WHEEL_CENTER_WIDTH),
        new Translation2d(-WHEEL_CENTER_DEPTH, WHEEL_CENTER_WIDTH));
      
    //angular offsets of module relative to chassis (rad)
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = 0; //spinning backword try: 3pi/2
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 3 * Math.PI/2; //still off by pi
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI/2; //spinning backword
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI; // try pi/2
    //backright old -1 * (Math.PI/2)
    //SPARK MAX CAN IDs987ytfdxz
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 6;
    public static final int FRONT_LEFT_TURNING_CAN_ID = 5;

    
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 3;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 4;
    
    public static final int BACK_LEFT_DRIVING_CAN_ID = 8;
    public static final int BACK_LEFT_TURNING_CAN_ID = 7;
   
    public static final int BACK_RIGHT_DRIVING_CAN_ID = 2;
    public static final int BACK_RIGHT_TURNING_CAN_ID = 1;
    
    public static final double INTAKE_ALIGN_KP = 0.0001;
    public static final double DIAGONAL_ALIGN_kP = 0.0001;

    public static double DRIVE_SPEED = 1;
    public static final double SLOW_MODE_MULTIPLIER = 0.5;
    public static final double AUTO_ALIGN_PID = 0.03;
    public static final double AUTO_ALIGN_MAX_SPEED = 1.00;

    public static final double shooterSideOffset = Units.inchesToMeters(6.0);

    public static final Transform2d shooterTransform = new Transform2d(Units.inchesToMeters(0.0), shooterSideOffset, new Rotation2d());
    public static final Pose3d redHubPose = new Pose3d(Units.inchesToMeters(485.5), Units.inchesToMeters(158.32), Units.inchesToMeters(72.0), new Rotation3d());
    public static final Pose3d blueHubPose = new Pose3d(Units.inchesToMeters(196), Units.inchesToMeters(158.32),  Units.inchesToMeters(72.0), new Rotation3d());

    public static final Pose3d realRedHubPose = new Pose3d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84), Units.inchesToMeters(72.0), new Rotation3d());
    public static final Pose3d realBlueHubPose = new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),  Units.inchesToMeters(72.0), new Rotation3d());
    public static final Pose3d redFerryPoseDepot = new Pose3d(14.3, 6, 0, Rotation3d.kZero);
    public static final Pose3d redFerryPoseOutpost = new Pose3d(14.3, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseDepot = new Pose3d(2.1, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseOutpost = new Pose3d(2.1, 6, 0, Rotation3d.kZero);

    public static final Angle epsilonAngleToGoal = Degrees.of(1.0);

    public static final Pose3d getHubPose() {
      
      Pose3d pose; 

      if (Constants.getCurrentAlliance() == Alliance.Blue) {
         pose = realBlueHubPose; 
      } else {
         pose = realRedHubPose; 
      }      
      //Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
      return pose;
    }

    public static final Pose3d getFerryPose(Translation2d robotPose) {
        if(DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            if (robotPose.getDistance(redFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose.getDistance(redFerryPoseOutpost.getTranslation().toTranslation2d())) {
                return redFerryPoseOutpost;
            } else {
                return redFerryPoseDepot;
            }
        } else {
            if (robotPose.getDistance(blueFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose.getDistance(blueFerryPoseOutpost.getTranslation().toTranslation2d())) {
                return blueFerryPoseOutpost;
            } else {
                return blueFerryPoseDepot;
            }
        }
    }
    public static final PIDController rotationController = getRotationController();

    public static double ROTATION_KP = 20;
    public static final double KP_INCREMENT = 0.01;


    private static final PIDController getRotationController() {
        PIDController controller = new PIDController(ROTATION_KP, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }

  
    public static final double POSE_ESTIMATOR_N1 = 3;
    public static final double POSE_ESTIMATOR_N2 = 3;

    public static final double POSE_ESTIMATOR_2_N1 = 1;
    public static final double POSE_ESTIMATOR_2_N2 = 1;

    public static final double VISION_STD_PTG1_N1 = 0.1;
    public static final double VISION_STD_PTG1_N2 = 0.1;

    public static final double VISION_STD_PTG2_N1 = 0.0001;
    public static final double VISION_STD_PTG2_N2 = 0.0001;


  }

  public static final class ModuleConstants{
    // The MAXSwerve module 3 pinion ggears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (more teeth = faster)
    public static final int DRIVING_MOTOR_PINION_TEETH = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = MotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

  }
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
    public static final double DRIVE_DEADBAND = 0.0005; 
  }

  

  

  public static final class MotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  } 

  
  
  public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 15;
        public static final int INTAKE_CURRENT_LIMIT = 40;
        public static final double INTAKING_POWER = 1; 
        public static final double INTAKING_REVERSE_POWER = 0.5;
        
        
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_LEADER_CANID = 16;
        public static final int SHOOTER_FOLLOWER_CANID = 17;
        
        public static final double POWEROFFSET_INCREMENT = 0.05;
        public static final double SHOOTER_BASE_POWER = 0.7;

        public static final int SHOOTER_CURRENT_LIMIT = 30;
        

    }
    //intake.retract(7s);;; [
    //  intake.donot.goout;;;;
    //];;;;
    //this is nathans code, yet to be tested

    public static final class HopperConstants {
      public static final double HOPPER_FAST_SPEED = 1;
      public static final double BELT_FAST_SPEED = 1;
      public static final double HOPPER_SLOW_SPEED = 0.5;
      public static final double BELT_SLOW_SPEED = 0.5;
      public static final double OUTTAKE_HOPPER_SPEED = -0.5;

      public static final int HOPPER_CURRENT_LIMIT = 25;

      // TODO: update these once krillpler is built
      public static final int TOP_ROLLERS_ID = 13;
      public static final int TOP_BELTS_ID = 14;
      public static final int ROLLER_FLOOR_ID = 10; 
      public static final int INDEXER_ID = 11;
    }

    public static final class UnjamSequenceConstants{
      public static final double FORWARD_TIME = 0.5;
      public static final double BACKWARD_TIME = 0.2;
      public static final double BUFFER_TIME = 0.01;
    }

    




    /*
     * if eshan_is_stooooopid True
     * /import Eshan_toucher_3000
     * 'punch'
     * 
     * 
     * ^^ben's code, also yet to be tested
     */

}

   

