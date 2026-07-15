package frc.robot;

import java.util.Optional;

import javax.net.ssl.TrustManagerFactory;
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
    public static final double DRIVE_DEADBAND = 0.05; 
  }

  public static final class AutoConstants {
    //add constants here that are not in pathplanner/limelight if needed
    public static final double X_TAG_ALIGNMENT_P = 0.01; 
    public static final double Y_TAG_ALIGNMENT_P = 0.1; 
    public static final double ROT_TAG_ALIGNMENT_P = 0.1; 

    public static final double ROT_SETPOINT_TAG_ALIGNMENT = 0;  //  RY Rotation
    public static final double ROT_TOLERANCE_TAG_ALIGNMENT = 1;
    
    public static final double X_SETPOINT_TAG_ALIGNMENT = 9.79;  //tx Vertical pose and tolerance 
    public static final double X_TOLERANCE_TAG_ALIGNMENT = 0.02;

    public static final double Y_SETPOINT_TAG_ALIGNMENT = -0.19;  // tz Horizontal pose (- for diff sides of tag)
    public static final double Y_TOLERANCE_TAG_ALIGNMENT = 0.02; 

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;
  }

  public static final class VisionConstants{
   public static final double LL_MOUNT_ANGLE_DEG = 0; //a1: degrees rotated up from vertical 
   public static final double LL_LENS_HEIGHT_IN = 7.1; //h1: distance from lens to floor 
   public static final double TARGET_HEIGHT_IN = 12.5;//44.25; //h2: height of target 
   public static final double robotToCameraX = 0;//X distance from center of robot to camera
   public static final double robotToCameraY = 0;//Y distance from center of robot to camera
   public static final double robotToCameraZ = 0;//Z distance from center of robot to camera
  }

  public static final class MotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  } 

  public static final class CalcConstants { // Bahhhh all the comments are going to fry me...
      //  FIELD GEOMETRY
      //  Where the target is on the field. I would like to pull this from the existing hub position if possible.
      // I do not want to make 2 different hubs for no reason. So if we could change this for later that would be great.
      // TODO: change this to reference the real hub pose instead of hardcoding it here.
  
      // Hug coordinates. Please tune. In METERS
      public static final Translation2d HUB_POSITION = new Translation2d(8.23, 4.11);
  
      // Height of the target opening above the floor, METRES. TODO: Tune this.
      public static final double TARGET_HEIGHT_METERS = 2.64;
  
      // Height of the shooter exit above the floor, METRES. TODO: Tune this.
      public static final double SHOOTER_HEIGHT_METERS = 0.60;


      //  TURRET CONSTRAINTS  (the wiring limit -- it canNOT spin forever)
      //  All angles in DEGREES, turret-relative (0 = pointing straight forward
      //  off the chassis). The calculator clamps every output into this range.
  
      // Most negative angle the turret can mechanically/safely reach. TODO: Tune this.
      public static final double TURRET_MIN_DEG = -200.0;
  
      // Most positive angle the turret can reach. TODO: Tune this. 
      public static final double TURRET_MAX_DEG = 200.0;
  
      /** A soft buffer kept away from each hard limit, DEGREES. The calculator
       *  will not command an angle inside this buffer, so the turret never slams
       *  the hard stop / strains the wiring. TODO: Tune this. */
      public static final double TURRET_SAFETY_BUFFER_DEG = 5.0;

  
      //  HOOD CONSTRAINTS  (it canNOT extend infinitely) -- Lmk if you prefer CANNOT or canNOT
      //  Servo position is the [0,1] command; angle is the physical launch angle
      //  that position produces. The map between them is assumed LINEAR; if the
      //  hood is on a linkage and is NOT linear, replace the linear map in the
      //  calculator with a measured lookup table.
  
      // Launch angle at servo position 0.0, DEGREES. TUNE (measure it). 
      public static final double HOOD_MIN_ANGLE_DEG = 20.0;
  
      // Launch angle at servo position 1.0, DEGREES. TUNE (measure it). 
      public static final double HOOD_MAX_ANGLE_DEG = 70.0;
  
      // Lowest servo position the hood is allowed to command, [0,1]. TODO: Tune this. Please.
      public static final double HOOD_MIN_SERVO = 0.0;
  
      // Highest servo position the hood is allowed to command, [0,1]. TODO: Tune this.
      public static final double HOOD_MAX_SERVO = 1.0;
  

      //  FLYWHEEL / SHOOTER GEOMETRY
      //  Used to convert a required launch velocity (m/s) into a flywheel RPM.
  
      // Flywheel wheel radius, METRES. TODO: Tune this. Again...
      public static final double WHEEL_RADIUS_METERS = 0.0508; // ~2 in, example. Tune again
  
      /** Gear ratio between motor and flywheel wheel (motor turns : wheel turns).
       *  1.0 if direct drive. TODO: Tune this. Again.... */
      public static final double FLYWHEEL_GEAR_RATIO = 1.0;
  
      /** Efficiency factor: real exit speed is lower than ideal wheel surface
       *  speed because the ball slips/compresses. 1.0 = perfect, ~0.5-0.8 typical.
       *  TODO: Tune this on the real shooter. */
      public static final double SHOOTER_EFFICIENCY = 1.0;
  
      //  SHOT VALIDITY  (when is a firing solution even reasonable?)
      //  Distances in METRES.
  
      /** Closest distance a shot is considered valid, METRES. TODO: Tune this. */
      public static final double MIN_SHOT_DISTANCE = 0.5;
  
      /** Farthest distance a shot is considered valid, METRES. TODO: Tune this. */
      public static final double MAX_SHOT_DISTANCE = 8.0;


      //  LOOKUP TABLES  (distance METRES -> RPM, and distance METRES -> hood servo)
      //  VALUES MUST BE RE-MEASURED for the new robot. The arrays are {distance, value} pairs.
      //  The calculator loads these into InterpolatingDoubleTreeMaps.
  
      // {distanceMeters, rpm}. TODO: Tune every row on the real robot. 
      public static final double[][] RPM_TABLE = {
          {1.0, 2850.0},
          {1.5, 3300.0},
          {2.0, 3500.0},
          {2.5, 3650.0},
          {3.0, 3800.0},
          {4.0, 4000.0},
      };
  
      // Did I forget to mention TODO: TUNE!!
      public static final double[][] HOOD_TABLE = {
          {1.0, 0.90},
          {1.5, 0.60},
          {2.0, 0.45},
          {2.5, 0.35},
          {3.0, 0.30},
          {4.0, 0.20},
      };
    }
  
  public static final class IntakeConstants {
        public static final int INTAKE_LEADER_ID = 15;
        public static final double INTAKING_FULL_POWER = 0.5; 
        public static final double INTAKING_PARTIAL_POWER = 0.2;
        public static final int LEADER_FORWARD_CHANNEL = 0;
        public static final int FOLLOWER_FORWARD_CHANNEL = 0;
        public static final int LEADER_REVERSE_CHANNEL = 1;
        public static final int FOLLOWER_REVERSE_CHANNEL = 1;
        
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_LEADER_CANID = 16;
        public static final int SHOOTER_FOLLOWER_CANID = 17;
        //public static final int SHOOTER_FEEDER_LEADER_CANID = 16; 
        //public static final int SHOOTER_FEEDER_FOLLOWER_CANID = 17;// updated from rev hardware client 2

        // PIDF Values
        public static final double LEADER_Kp = 0.0000709999; // 0.00061;
        public static final double LEADER_Kd = 0; //0.00003;
        public static final double LEADER_Ki = 0; //0.000019998;
        public static final double LEADER_FF_kS = 0.0;
        public static final double LEADER_FF_kV = 0.00012;
        public static final double LEADER_FF_kA = 0.0002;

        
    
        // Manual Control 
        public static final double RPM_INCREMENT = 12.5;
        public static final double KV_INCREMENT = 0.000001;
        public static final double KP_INCREMENT = 0.001;
        public static final double KI_INCREMENT = 0.0001;
        public static final double KD_INCREMENT = 0.0001;
        public static final double HUB_RPM = -4000;
        public static final Rotation2d HUB_RPM_ANGLE = Rotation2d.fromDegrees(20);

        
        ;
        

        public static final double VELOCITY_TOLERANCE =  30; 
        

        

        public static final double RPMOFFSET_INCREMENT = 200;
        //hood constants 

        public static final double HOOD_ANGLE_TO_SERVO_MULTIPLIER = 1.909859317;
        public static final Rotation2d MIN_HOOD_ANGLE = Rotation2d.fromDegrees(16.172); 
        

        //untested kinematics equation
                   // 0.80–0.95, tune this

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

      // TODO: update these once krillpler is built
      public static final int TOP_ROLLERS_ID = 13;
      public static final int TOP_BELTS_ID = 14;
      public static final int ROLLER_FLOOR_ID = 10; 
      public static final int INDEXER_ID = 11;
    }

    public static final class TurretConstants {
      //TODO: correct these once krillpler is wired
      public static final int TURRET_MOTOR_ID = 0;
      public static final double TURRET_LOOP_POINT = 0; //measured in radians on domain (0, 2pi)
      // 0 is turret facing forwards
      public static final float TURRET_KP = 0;
      public static final float TURRET_KD = 0;
      public static final float TURRET_KI = 0;

      public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATION = 11/128;


      
      
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

   

