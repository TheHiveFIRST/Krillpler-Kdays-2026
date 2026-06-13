// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.Supplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.configs.TurretConfig;

/**
 * Turret subsystem -- "krilldih'.
 *
 * Hardware:
 *   - Kraken X60 (TalonFX) for azimuth, using its built-in motor-shaft encoder.
 *   - SOmething for the hood I think. Not sure.
 *   - NEO Vortex (SparkFlex) for the flywheel.
 *
 * Design notes carried over from the FTC Sharkbuh:
 *   - The turret does NOT run vision. It asks the drivetrain's pose estimator
 *     (which fuses odometry + PhotonVision) for the robot pose. See the suppliers.
 *   - The flywheel uses honest bang-bang, not three controllers fighting.
 *   - The Kraken encoder is on the MOTOR SHAFT, before the gearbox. That means
 *     (a) gear backlash is between the encoder and the real turret, and
 *     (b) it reads 0 wherever it powers up. You must HOME it. See seedTurretZero().
 */
public class TurretSubsystem extends SubsystemBase {

    //  ENUM

    public enum StartPosition {
        RedStart1, RedStart2, RedStart3,
        BlueStart1, BlueStart2, BlueStart3,
        NoPosition
    }

    // Goon
    //  HARDWARE
    //  Declared and constructed here. They are CONFIGURED in the constructor.
    //  CAN IDs come from Constants -- coordinate them with the drivetrain so no
    //  two devices collide.

    private final TalonFX turretMotor =
            new TalonFX(TurretConstants.TURRET_CAN_ID); // Replace later

    private final SparkFlex shooterMotor =
            new SparkFlex(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless); // Replace Later

    // A reusable control request object for MotionMagic position control.
    // You set its target each loop rather than making a new one every time.
    private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);

    //  GOAL COORDINATES (field frame, METRES -- FRC field math is metric)
    private final Translation2d B1 = new Translation2d(TurretConstants.B1_X, TurretConstants.B1_Y);
    private final Translation2d B2 = new Translation2d(TurretConstants.B2_X, TurretConstants.B2_Y);
    private final Translation2d B3 = new Translation2d(TurretConstants.B3_X, TurretConstants.B3_Y);
    private final Translation2d R1 = new Translation2d(TurretConstants.R1_X, TurretConstants.R1_Y);
    private final Translation2d R2 = new Translation2d(TurretConstants.R2_X, TurretConstants.R2_Y);
    private final Translation2d R3 = new Translation2d(TurretConstants.R3_X, TurretConstants.R3_Y);
    private final Translation2d NO_POSITION = new Translation2d(0, 0);


    //  Key = distance to goal in metres. Values filled in populateLookupTables().
    private final InterpolatingDoubleTreeMap rpmTable  = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();


    //  This is why the turret never touches vision: someone else produces the pose.
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;


    //  STATE -- not final, these change while the robot runs.
    private StartPosition currentPosition = StartPosition.NoPosition;
    private Translation2d targetGoal = NO_POSITION;
    private boolean shooterEnabled = false;
    private double targetShooterRPM = 0.0;
    private double lastTurretTargetDeg = 0.0;   // kept for isOnTarget() + telemetry
    private double lastDistanceMeters  = 0.0;

    //  CONSTRUCTOR
    //  Runs ONCE when the subsystem is created. Stores the suppliers, then does
    public TurretSubsystem(Supplier<Pose2d> poseSupplier,
                            Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;

        configureTurretMotor();
        configureShooterMotor();
        populateLookupTables();
    }

    //  CONFIGURATION METHODS

    
    private void configureTurretMotor() {
        turretMotor.getConfigurator().apply(TurretConfig.TurretConfigs.trackHub);
    }


    private void configureShooterMotor() {
        SparkFlexConfig shooterConfig = new SparkFlexConfig();

        shooterConfig.idleMode(IdleMode.kCoast);
        shooterConfig.smartCurrentLimit(40);
        
        shooterConfig.closedLoop.p(0.0); // Remember this shi too. 
        shooterConfig.closedLoop.i(0.0);
        shooterConfig.closedLoop.d(0.0);
        shooterConfig.closedLoop.velocityFF(0.0);


        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void populateLookupTables() {
        // These are EMPIRICAL -- measured on the real robot, not calculated.

        // TODO figure these out pls.

        rpmTable.put(0.0, 0.0);
        hoodTable.put(0.0, 0.0);
    }

    // ========================================================================
    //  HOMING
    //  The Kraken encoder is relative -- it reads 0 wherever it boots. Before the
    //  turret angle means anything, you must tell the motor "you are at zero now".
    // ========================================================================

    /**
     * TODO: Seed the turret's zero.
     *  - Physically: rotate the turret to its known mechanical straight-ahead
     *    position (a hard stop or witness mark makes this repeatable).
     *  - In code: turretMotor.setPosition(0); tells the encoder "this is zero".
     *  - Call this during setup / from a button before each match.
     *  - BETTER (free upgrade if you have a spare limit switch): wire a switch at
     *    the zero position, drive slowly to it on boot, then setPosition(0).
     *    That removes the "someone forgot to home it" failure mode entirely.
     */
    public void seedTurretZero() {
        // TODO
    }

    // ========================================================================
    //  MAIN LOOP -- the scheduler calls this every 20 ms. You never call it.
    // ========================================================================
    @Override
    public void periodic() {

        // --- STEP 1: read the robot's current state -------------------------
        // TODO: get the robot Pose2d from poseSupplier, and the field-relative
        //       ChassisSpeeds from fieldRelativeSpeedsSupplier.

        // --- STEP 2: lead the shot (the "virtual goal" from your FTC code) ---
        // TODO:
        //   - distance from robot to targetGoal  (Translation2d has .getDistance())
        //   - flight time = estimateTimeOfFlight(thatDistance)
        //   - virtualGoal = targetGoal shifted BACKWARDS along the robot's
        //     velocity by (velocity * flightTime). Build a new Translation2d:
        //       x = targetGoal.getX() - speeds.vxMetersPerSecond * tof
        //       y = targetGoal.getY() - speeds.vyMetersPerSecond * tof

        // --- STEP 3: azimuth math (same as FTC solveFiringSolution) ----------
        // TODO:
        //   - vector from robot to virtualGoal: virtualGoal.minus(robotTranslation)
        //   - field-relative angle of that vector: .getAngle().getDegrees()
        //   - turret-relative angle = fieldAngle - robotHeadingDegrees
        //   - wrap into [-180, 180] with MathUtil.inputModulus(...)
        //   - clamp into the turret's mechanical range with MathUtil.clamp(...)
        //   - store the result in lastTurretTargetDeg (telemetry/isOnTarget need it)

        // --- STEP 4: command the turret -------------------------------------
        // TODO:
        //   - convert your target ANGLE (degrees) into MOTOR ROTATIONS using
        //     turretDegreesToMotorRotations(...) -- the gear ratio matters here.
        //   - set the target on turretRequest: turretRequest.withPosition(rotations)
        //   - send it: turretMotor.setControl(turretRequest);
        //   MotionMagic handles the profile + PID + feedforward onboard for you.

        // --- STEP 5: flywheel -- honest bang-bang ---------------------------
        // TODO:
        //   if shooterEnabled:
        //     - targetShooterRPM = rpmTable.get(lastDistanceMeters)
        //     - if currentRPM < targetShooterRPM  -> shooterMotor.set(1.0)
        //       (full voltage below target = fastest possible recovery)
        //     - else                              -> shooterMotor.set(SHOOTER_KF)
        //       (a small floor so it holds speed instead of coasting down)
        //   else:
        //     - shooterMotor.set(0.0)

        // --- STEP 6: telemetry ----------------------------------------------
        publishTelemetry();
    }

    // ========================================================================
    //  HELPER METHODS
    // ========================================================================

    /**
     * TODO: Estimate projectile flight time for the moving-shot lead.
     *  - For a first pass, a crude linear model is fine: distance * someConstant.
     *  - LATER: replace with your Ballistics class. First check whether its
     *    methods are static -- that decides if you need a Ballistics object.
     */
    private double estimateTimeOfFlight(double distanceMeters) {
        // TODO
        return 0.0; // placeholder so the skeleton compiles
    }

    /**
     * TODO: Convert a turret angle in DEGREES to MOTOR rotations.
     *  - The motor turns GEAR_RATIO times for each 1 turn of the turret.
     *  - degrees -> turret rotations (/360) -> motor rotations (* GEAR_RATIO).
     */
    private double turretDegreesToMotorRotations(double degrees) {
        // TODO
        return 0.0; // placeholder
    }

    /**
     * TODO: Convert MOTOR rotations back to turret DEGREES. The inverse of above.
     */
    private double motorRotationsToTurretDegrees(double motorRotations) {
        // TODO
        return 0.0; // placeholder
    }

    /**
     * TODO: Return the turret's current angle in DEGREES.
     *  - turretMotor.getPosition() gives a signal; .getValueAsDouble() reads it,
     *    in MOTOR rotations.
     *  - Feed that through motorRotationsToTurretDegrees(...).
     */
    public double getTurretAngleDegrees() {
        // TODO
        return 0.0; // placeholder
    }

    /**
     * TODO: Return the flywheel speed in RPM.
     *  - shooterMotor.getEncoder().getVelocity() returns RPM directly for a Spark.
     */
    public double getShooterRPM() {
        // TODO
        return 0.0; // placeholder
    }

    // ========================================================================
    //  PUBLIC API -- how commands / RobotContainer talk to this subsystem.
    // ========================================================================

    /**
     * TODO: Set currentPosition and pick the matching targetGoal.
     *  - A switch on the StartPosition enum mapping each value to B1..R3.
     *  - Default / NoPosition -> NO_POSITION.
     */
    public void setStartPosition(StartPosition position) {
        // TODO
    }

    /** TODO: set shooterEnabled = true. */
    public void spinUpShooter() {
        // TODO
    }

    /** TODO: set shooterEnabled = false. */
    public void stopShooter() {
        // TODO
    }

    /**
     * TODO: return true when the turret is pointed within tolerance of target.
     *  - compare lastTurretTargetDeg to getTurretAngleDegrees().
     *  - tolerance: TurretConstants.ON_TARGET_TOLERANCE_DEG.
     */
    public boolean isOnTarget() {
        // TODO
        return false; // placeholder
    }

    /**
     * TODO: return true when the flywheel is within tolerance of target RPM.
     *  - only meaningful if shooterEnabled and targetShooterRPM > 0.
     *  - tolerance: ShooterConstants.RPM_TOLERANCE.
     */
    public boolean isShooterAtSpeed() {
        // TODO
        return false; // placeholder
    }

    /**
     * TODO: return true only when BOTH isOnTarget() and isShooterAtSpeed().
     *  - This is what your feeder/indexer should gate every ball on.
     */
    public boolean isReadyToShoot() {
        // TODO
        return false; // placeholder
    }

    // ========================================================================
    //  TELEMETRY -- push values to the dashboard so you can see + tune.
    // ========================================================================

    /**
     * TODO: SmartDashboard.putNumber / putBoolean / putString for the things you
     * want to watch: turret angle, target angle, distance, shooter RPM vs target,
     * isOnTarget, isReadyToShoot. You are blind without this when tuning.
     */
    private void publishTelemetry() {
        // TODO
    }
}