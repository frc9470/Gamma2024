package com.team9470.shooter;


import com.team254.lib.util.InterpolatingDouble;
import com.team9470.FieldLayout;
import com.team9470.TunableNumber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A data class to represent shot parameters for a certain shot.
 */
public record ShotParameters(double distance, double rpm, double angle, Rotation2d heading) implements Sendable {

    private static final Pose2d SPEAKER_CENTER = FieldLayout.kSpeakerCenter;
    private static final double TO_F_FACTOR = 0.2;
    private static final boolean tuning = false;

    private static final TunableNumber rpmTune = new TunableNumber("Shooter/rpm", 0.0, tuning);
    private static final TunableNumber angleTune = new TunableNumber("Shooter/angle", 45, tuning);
    private static final TunableNumber yawTune = new TunableNumber("Shooter/yaw", 0.0, tuning);

    /**
     * Calculates the shot parameters based on the provided parameters.
     *
     * @param pose         The current pose of the robot.
     * @param velocity     The current velocity of the robot.
     * @param isRedAlliance Whether the robot is on the red alliance.
     * @param shootOnMove  Whether the robot is shooting while moving.
     * @return A ShotParameters instance containing the calculated shot parameters.
     */
    public static ShotParameters calculate(Pose2d pose, Twist2d velocity, boolean isRedAlliance, boolean shootOnMove) {
        Translation2d target = FieldLayout.handleAllianceFlip(SPEAKER_CENTER.getTranslation(), isRedAlliance);
        Translation2d targetRelative = target.minus(pose.getTranslation());

        double yaw = targetRelative.getAngle().getDegrees();
        double distance = targetRelative.getNorm();
        SmartDashboard.putNumber("FiringParams/Uncomped Dist", distance);
        double range;

        if (tuning) return new ShotParameters(distance, rpmTune.get(), angleTune.get(), new Rotation2d(yawTune.get()));

        if (shootOnMove) {
            double[] adjustedParams = adjustForMovement(yaw, distance, velocity);
            yaw = adjustedParams[0];
            range = adjustedParams[1];
        } else {
            range = distance;
        }

        double rpm = getShooterSpeed(range);
        double angle = getShooterAngle(range);
        double heading = yaw + getShooterYaw(range);
//        System.out.println(yaw);
//        System.out.println(getShooterYaw(range));

        return new ShotParameters(distance, rpm, angle, Rotation2d.fromDegrees(heading));
    }

    public static ShotParameters simple(double distance, double yaw, boolean isRedAlliance){
        Translation2d target = FieldLayout.handleAllianceFlip(SPEAKER_CENTER.getTranslation(), isRedAlliance);

        double range;

        if (tuning) return new ShotParameters(distance, rpmTune.get(), angleTune.get(), new Rotation2d(yawTune.get()));

        range = distance;

        double rpm = getShooterSpeed(range);
        double angle = getShooterAngle(range);
        double heading = yaw + getShooterYaw(range);

        return new ShotParameters(distance, rpm, angle, new Rotation2d(heading));
    }

    /**
     * Adjusts the yaw and range based on the robot's movement.
     *
     * @param yaw            The initial yaw.
     * @param distance       The initial distance.
     * @param velocity       The current velocity of the robot.
     * @return An array containing the adjusted yaw and range.
     */
    private static double[] adjustForMovement(double yaw, double distance, Twist2d velocity) {
        Translation2d polarVelocity = new Translation2d(velocity.dx, velocity.dy).rotateBy(Rotation2d.fromDegrees(yaw));
        double radial = polarVelocity.getX();
        double tangential = polarVelocity.getY();

        SmartDashboard.putNumber("FiringParams/Tangential", tangential);
        SmartDashboard.putNumber("FiringParams/Radial", radial);

        double shotSpeed = distance / TO_F_FACTOR - radial;
        shotSpeed = Math.max(0.0, shotSpeed);
        yaw += Units.radiansToDegrees(Math.atan2(-tangential, shotSpeed));
        double range = TO_F_FACTOR * Math.sqrt(Math.pow(tangential, 2) + Math.pow(shotSpeed, 2));

        return new double[]{yaw, range};
    }

    /**
     * Gets the shooter speed based on the range.
     *
     * @param range The distance to the target.
     * @return The shooter speed.
     */
    public static double getShooterSpeed(double range) {
        return RegressionMaps.SPEED_MAP.getInterpolated(new InterpolatingDouble(range)).value;
    }

    /**
     * Gets the shooter angle based on the range.
     *
     * @param range The distance to the target.
     * @return The shooter angle.
     */
    public static double getShooterAngle(double range) {
        return RegressionMaps.ANGLE_MAP.getInterpolated(new InterpolatingDouble(range)).value;
    }

    /**
     * Gets the shooter yaw based on the range.
     *
     * @param range The distance to the target.
     * @return The shooter yaw.
     */
    public static double getShooterYaw(double range) {
        return RegressionMaps.YAW_MAP.getInterpolated(new InterpolatingDouble(range)).value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty("params", () -> new double[]{distance, rpm, angle, heading.getDegrees()}, (double[] d) -> {});
    }
}