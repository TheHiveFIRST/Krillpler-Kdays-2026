// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.global.Calculation;

/**
 * converted to SI units (METRES, METRES/SEC, SECONDS, DEGREES in / radians
 * internal).
 * NOTE: real projectile flight has air drag, ball spin, and compression that
 * these clean equations do NOT model. Use these for time-of-flight and as a
 * STARTING estimate; the empirical lookup tables in the calculator are what make
 * the actual shot land.
 */

public final class Ballistics {

    private Ballistics() {} // pure static utility, never instantiate

    /** Gravity, METRES per second squared */
    public static final double GRAVITY = 9.80665;

    /**
     * Required launch velocity to hit a target at a given horizontal distance
     * and height difference, for a fixed launch angle. Standard projectile
     * kinematics.
     *
     * @param distanceMeters horizontal distance to target (m)
     * @param heightDiffMeters targetHeight - shooterHeight (m)
     * @param launchAngleDeg physical launch angle (deg)
     * @return required launch velocity (m/s), or 0 if the shot is physically
     *         impossible at that angle (target too high for the angle).
     */
    public static double requiredLaunchVelocity(double distanceMeters,
                                                double heightDiffMeters,
                                                double launchAngleDeg) {
        double theta = Math.toRadians(launchAngleDeg);
        double x = distanceMeters;
        double y = heightDiffMeters;

        double numerator = GRAVITY * x * x;
        double denominator = 2.0 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y);

        // Negative/zero denominator => target unreachable at this angle.
        if (denominator <= 0) {
            return 0.0;
        }
        return Math.sqrt(numerator / denominator);
    }

    /**
     * How long the projectile is in the air. Used for the moving-shot lead.
     *
     * @param distanceMeters horizontal distance (m)
     * @param launchVelocityMps launch velocity (m/s)
     * @param launchAngleDeg launch angle (deg)
     * @return time of flight (seconds), or 0 if velocity/angle make it invalid.
     */
    public static double timeOfFlight(double distanceMeters,
                                      double launchVelocityMps,
                                      double launchAngleDeg) {
        double theta = Math.toRadians(launchAngleDeg);
        double vx = launchVelocityMps * Math.cos(theta);
        if (vx <= 1e-9) {
            return 0.0; // avoid divide-by-zero on a vertical/zero shot
        }
        return distanceMeters / vx;
    }

    /**
     * Convert a desired ball exit velocity (m/s) into a flywheel motor RPM.
     *
     * @param exitVelocityMps desired ball exit speed (m/s)
     * @param wheelRadiusMeters flywheel wheel radius (m)
     * @param gearRatio motor-turns : wheel-turns (1.0 if direct)
     * @param efficiency fraction of ideal surface speed actually imparted (0..1)
     * @return required motor RPM
     */
    public static double exitVelocityToMotorRpm(double exitVelocityMps,
                                                double wheelRadiusMeters,
                                                double gearRatio,
                                                double efficiency) {
        if (wheelRadiusMeters <= 1e-9 || efficiency <= 1e-9) {
            return 0.0;
        }
        // Ball speed is (efficiency) * wheel surface speed.
        double wheelSurfaceSpeed = exitVelocityMps / efficiency;
        double circumference = 2.0 * Math.PI * wheelRadiusMeters;
        double wheelRpm = (wheelSurfaceSpeed * 60.0) / circumference;
        return wheelRpm * gearRatio;
    }

    /**
     * Convert a flywheel motor RPM into the ball exit velocity (m/s) it produces.
     * The inverse of {@link #exitVelocityToMotorRpm}. Useful for time-of-flight
     * when I got RPM from the lookup table.
     */
    public static double motorRpmToExitVelocity(double motorRpm,
                                                double wheelRadiusMeters,
                                                double gearRatio,
                                                double efficiency) {
        if (gearRatio <= 1e-9) {
            return 0.0;
        }
        double wheelRpm = motorRpm / gearRatio;
        double circumference = 2.0 * Math.PI * wheelRadiusMeters;
        double wheelSurfaceSpeed = (wheelRpm * circumference) / 60.0;
        return wheelSurfaceSpeed * efficiency;
    }
}