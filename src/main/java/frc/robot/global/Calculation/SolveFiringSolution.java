// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.global.Calculation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants.CalcConstants;

/**
 * READ ME
 * Every public method is PURE: inputs as parameters, result returned, no
 * reading of sensors, no motor commands, no stored mutable state. The two
 * lookup tables are built once and only read after.
 * This class NEVER controls the turret. No PID, no setVoltage, no motors.
 * Marshall doing the turret consumes {@link FiringSolution#turretAngleDeg}
 * and drives the motor themselves.
 * Units are SI everywhere: METRES, METRES/SEC, DEGREES, RPM.
 *
 * HOW THE TURRET USES IT:
 *   FiringSolution sol = calc.solve(drive.getPose(), drive.getFieldRelativeSpeeds());
 *   if (sol.validShot) {
 *       turret.setTargetAngle(sol.turretAngleDeg);   // Marshall code
 *       hood.setPosition(sol.hoodServoPos);          // Marshall code
 *       shooter.setTargetRpm(sol.flywheelRpm);       // Marshall code
 *   }
 */
public class SolveFiringSolution {

    // distance (m) -> flywheel RPM 
    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
    // distance (m) -> hood servo position [0,1] 
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();

    /** The field location being aimed at. Defaults to the hub from constants;
     *  the caller can override (e.g. with the team's alliance-aware hub pose). */
    private final Translation2d target;

    // Build with the default hub target from constants. 
    public SolveFiringSolution() {
        this(CalcConstants.HUB_POSITION);
    }

    // Build aiming at a specific field point (e.g. DriveConstants.getHubPose()). 
    public SolveFiringSolution(Translation2d target) {
        this.target = target;
        loadTables();
    }

    // Loads the lookup tables from the constant arrays. Called once at build. 
    private void loadTables() {
        for (double[] row : CalcConstants.RPM_TABLE) {
            rpmTable.put(row[0], row[1]);
        }
        for (double[] row : CalcConstants.HOOD_TABLE) {
            hoodTable.put(row[0], row[1]);
        }
    }

    /**
     * Compute a full firing solution for a STATIONARY shot (no velocity lead).
     * @param robotPose fused robot pose from the drivetrain (field frame, metres)
     */
    public FiringSolution solve(Pose2d robotPose) {
        return solve(robotPose, new ChassisSpeeds()); // zero velocity = no lead
    }

    /**
     * Compute a full firing solution, leading the shot for robot motion.
     *
     * @param robotPose fused robot pose (field frame, metres)
     * @param fieldRelativeSpeeds robot velocity in the FIELD frame (m/s).
     *        If you only have robot-relative speeds, convert first with
     *        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotPose.getRotation()).
     */
    public FiringSolution solve(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
        Translation2d robotXY = robotPose.getTranslation();

        // raw distance, for an initial time-of-flight estimate
        double rawDistance = robotXY.getDistance(target);

        // estimate flight time from the table's RPM at this distance 
        double estRpm = rpmTable.get(rawDistance);
        double estExitVel = Ballistics.motorRpmToExitVelocity(
                estRpm,
                CalcConstants.WHEEL_RADIUS_METERS,
                CalcConstants.FLYWHEEL_GEAR_RATIO,
                CalcConstants.SHOOTER_EFFICIENCY);
        double estLaunchAngle = hoodServoToAngleDeg(hoodTable.get(rawDistance));
        double tof = Ballistics.timeOfFlight(rawDistance, estExitVel, estLaunchAngle);

        // virtual goal: shift target back along robot velocity * tof   
        // (Ported from the FTC virtual-goal lead. With zero velocity this is a
        //  no-op and you get a normal stationary solution.)
        Translation2d virtualTarget = new Translation2d(
                target.getX() - fieldRelativeSpeeds.vxMetersPerSecond * tof,
                target.getY() - fieldRelativeSpeeds.vyMetersPerSecond * tof);

        // azimuth to the (virtual) target
        Translation2d toTarget = virtualTarget.minus(robotXY);
        Rotation2d fieldAngle = toTarget.getAngle();                 // field frame
        double turretRelDeg = fieldAngle.getDegrees()
                - robotPose.getRotation().getDegrees();              // turret frame
        turretRelDeg = MathUtil.inputModulus(turretRelDeg, -180.0, 180.0);

        // apply the turret's wiring limit 
        // The turret canNOT (CANNOT OR canNOT? lmk) spin forever. Clamp into the safe travel range, kept
        // a buffer away from the hard stops. NOTE: this clamps the COMMAND; if the
        // clamped angle no longer points at the target, the shot is marked invalid
        // below so nobody fires into a wall thinking they are aimed.
        double clampedTurretDeg = MathUtil.clamp(
                turretRelDeg,
                CalcConstants.TURRET_MIN_DEG + CalcConstants.TURRET_SAFETY_BUFFER_DEG,
                CalcConstants.TURRET_MAX_DEG - CalcConstants.TURRET_SAFETY_BUFFER_DEG);
        boolean turretReachable = Math.abs(clampedTurretDeg - turretRelDeg) < 1e-6;

        // distance to the virtual target, for the final RPM/hood 
        double virtualDistance = virtualTarget.minus(robotXY).getNorm();

        double rpm = rpmTable.get(virtualDistance);
        double hoodServo = MathUtil.clamp(
                hoodTable.get(virtualDistance),
                CalcConstants.HOOD_MIN_SERVO,
                CalcConstants.HOOD_MAX_SERVO);

        // shot validity 
        boolean inRange = virtualDistance >= CalcConstants.MIN_SHOT_DISTANCE
                && virtualDistance <= CalcConstants.MAX_SHOT_DISTANCE;
        boolean validShot = inRange && turretReachable;

        return new FiringSolution(clampedTurretDeg, hoodServo, rpm, virtualDistance, validShot);
    }

    /**
     * Hood servo position [0,1] -> launch angle (deg), assuming a LINEAR hood.
     * If the hood is on a non-linear linkage, replace this with a measured table.
     */
    public double hoodServoToAngleDeg(double servoPos) {
        double t = MathUtil.clamp(servoPos, 0.0, 1.0);
        // interpolate(start, end, t) -- t goes LAST.
        return MathUtil.interpolate(
                CalcConstants.HOOD_MIN_ANGLE_DEG,
                CalcConstants.HOOD_MAX_ANGLE_DEG,
                t);
    }

    /**
     * Launch angle (deg) -> hood servo position [0,1]. The inverse of the above;
     * this is the direction the shooter usually needs (ballistics give an angle,
     * the servo needs a position).
     */
    public double hoodAngleToServo(double angleDeg) {
        double servo = MathUtil.inverseInterpolate(
                CalcConstants.HOOD_MIN_ANGLE_DEG,
                CalcConstants.HOOD_MAX_ANGLE_DEG,
                angleDeg);
        return MathUtil.clamp(servo, CalcConstants.HOOD_MIN_SERVO,
                CalcConstants.HOOD_MAX_SERVO);
    }

    /** Straight-line distance from a robot pose to the target, metres. */
    public double distanceToTarget(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(target);
    }


    /**
     * The output of a calculation. Plain data -- Marshall
     * reads these fields and commands their own motors. Immutable on purpose.
     */
    public static final class FiringSolution {
        // Turret angle, DEGREES, turret-relative, already clamped to limits. 
        public final double turretAngleDeg;
        // Hood servo position, [0,1], already clamped. 
        public final double hoodServoPos;
        // Flywheel target speed, RPM. 
        public final double flywheelRpm;
        // Distance used for the solution, METRES (after velocity lead). 
        public final double distanceMeters;
        // True only if in range AND the turret can actually point there.   
        public final boolean validShot;

        public FiringSolution(double turretAngleDeg, double hoodServoPos,
                              double flywheelRpm, double distanceMeters,
                              boolean validShot) {
            this.turretAngleDeg = turretAngleDeg;
            this.hoodServoPos = hoodServoPos;
            this.flywheelRpm = flywheelRpm;
            this.distanceMeters = distanceMeters;
            this.validShot = validShot;
        }
    }
}
