//package com.team9470.shooter;
//
//import com.team254.lib.util.InterpolatingDouble;
//import com.team254.lib.util.Util;
//import com.team9470.FieldLayout;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//
//public class FerryUtil {
//
//	private static final double kOppoWingToAllianceWall =
//			FieldLayout.distanceFromAllianceWall(FieldLayout.kWingX, true);
//	public static final Translation2d kCornerTarget = new Translation2d(1.0, FieldLayout.kFieldWidth - 1.5);
//	public static final Translation2d kMidTarget =
//			new Translation2d((FieldLayout.kFieldLength / 2.0) - 1.0, FieldLayout.kFieldWidth - 0.2);
//
//	/**
//	 * Get adaptive ferry shot parameters
//	 * @param robot_pose Pose of the robot
//	 * @param is_red_alliance
//	 * @return Array of length 4 containing distance to target, hood angle, shooter rpm, and drivetrain angle
//	 */
//	public static double[] getFerryShotParameters(Pose2d robot_pose, boolean is_red_alliance) {
//		boolean midfield_target = useMidfieldTarget(robot_pose.getTranslation().getX(), is_red_alliance);
//		Translation2d target;
//		if (midfield_target) {
//			target = FieldLayout.handleAllianceFlip(kMidTarget, is_red_alliance);
//		} else {
//			target = FieldLayout.handleAllianceFlip(kCornerTarget, is_red_alliance);
//		}
//		Translation2d robot_to_target =
//				target.plus(robot_pose.getTranslation().unaryMinus());
//		Rotation2d target_drive_heading = robot_to_target
//				.getAngle()
//				.rotateBy(Rotation2d.fromDegrees(180.0))
//				.rotateBy(Rotation2d.fromDegrees(-10.0));
//		double shooter_setpoint, hood_setpoint, dist_to_target;
//		dist_to_target = robot_to_target.getNorm();
//		if (inHighFerryZone(robot_pose, is_red_alliance) || midfield_target) {
//			hood_setpoint =
//					RegressionMaps.kHighFerryHoodMap.getInterpolated(new InterpolatingDouble(dist_to_target)).value;
//			shooater_setpoint =
//					RegressionMaps.kHighFerryRPMMap.getInterpolated(new InterpolatingDouble(dist_to_target)).value;
//			target_drive_heading = target_drive_heading.rotateBy(Rotation2d.fromDegrees(0.0));
//		} else {
//			hood_setpoint =
//					RegressionMaps.kLowFerryHoodMap.getInterpolated(new InterpolatingDouble(dist_to_target)).value;
//			shooter_setpoint =
//					RegressionMaps.kLowFerryRPMMap.getInterpolated(new InterpolatingDouble(dist_to_target)).value;
//		}
//		return new double[] {dist_to_target, hood_setpoint, shooter_setpoint, target_drive_heading.getDegrees()};
//	}
//
//	private static boolean inHighFerryZone(Pose2d robot_pose, boolean is_red_alliance) {
//		double x = robot_pose.getTranslation().getX();
//		double y = robot_pose.getTranslation().getY();
//		Translation2d cor_0 =
//				FieldLayout.handleAllianceFlip(new Translation2d(FieldLayout.kWingX, 0.0), is_red_alliance);
//		Translation2d cor_1 = FieldLayout.kCenterNote2;
//		boolean in_x = Util.inRange(x, Math.min(cor_0.getX(), cor_1.getX()), Math.max(cor_0.getX(), cor_1.getX()));
//		boolean in_y = Util.inRange(y, Math.min(cor_0.getY(), cor_1.getY()), Math.max(cor_0.getY(), cor_1.getY()));
//		return in_x && in_y;
//	}
//
//	private static boolean useMidfieldTarget(double x_coord, boolean is_red_alliance) {
//		return FieldLayout.distanceFromAllianceWall(x_coord, is_red_alliance) - kOppoWingToAllianceWall > 1.0;
//	}
//}
