package com.team9470.subsystems.vision;

import com.team9470.FieldLayout;
import com.team9470.Robot;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ejml.equation.MatrixConstructor;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionDevice {

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


    public VisionDevice(String name, Transform3d transform) {
        this.photonCamera = new PhotonCamera(name);
        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                photonCamera,
                transform// Robot to camera transform (adjust as needed)

        );

    }

    public void updatePosition(Swerve swerve) {
        Optional<EstimatedRobotPose> posEstimate = getPosEstimate();
        if (posEstimate.isEmpty()) {
            return;
        }

        EstimatedRobotPose estimatedPose = posEstimate.get();
        Pose2d pose = estimatedPose.estimatedPose.toPose2d();
        double timestamp = estimatedPose.timestampSeconds;

        double std_dev_multiplier = 1.0;

        List<Pose3d> tagPoses = new ArrayList<>();

        // Add all tag poses to the list
        for (PhotonTrackedTarget target : photonCamera.getLatestResult().getTargets()) {
            int tagId = target.getFiducialId();
            Optional<Pose3d> tagPose = FieldLayout.kTagMap.getTagPose(tagId);
            tagPose.ifPresent(tagPoses::add);
        }

        if (tagPoses.isEmpty()) {
            return;
        }

        // Calculate distances and statistics
        Pair<Double, Double> distanceStats = calculateDistanceStatistics(tagPoses, pose);
        double lowestDist = distanceStats.getFirst();
        double avgDist = distanceStats.getSecond();

        // Estimate standard deviation of vision measurement
        double xyStdDev = calculateXYStandardDeviation(lowestDist, avgDist, tagPoses.size(), std_dev_multiplier);

        // Log vision data
//        logVisionData(tagPoses, xyStdDev, pose, timestamp);

        if (Vision.getInstance().isVisionDisabled()) {
            return;
        }

        // Update robot state with vision data
        swerve.addVisionMeasurement(pose, timestamp, new Matrix<>(Nat.N3(), Nat.N1(), new double[]{xyStdDev, xyStdDev, 0.02}));

        // Calculate and log rotation
        //double rotationDegrees = calculateRotation(pose, Swerve.isRedAlliance());
//        logRotation(rotationDegrees);

    }

    private Pair<Double, Double> calculateDistanceStatistics(List<Pose3d> tagPoses, Pose2d pose) {
        double totalTagDist = 0.0;
        double lowestDist = Double.POSITIVE_INFINITY;

        for (Pose3d pose3d : tagPoses) {
            double dist = pose3d.getTranslation().toTranslation2d().getDistance(pose.getTranslation());
            totalTagDist += dist;
            lowestDist = Math.min(dist, lowestDist);
        }

        double avgDist = totalTagDist / tagPoses.size();
        return new Pair<>(lowestDist, avgDist);
    }

    private double calculateXYStandardDeviation(double lowestDist, double avgDist, int tagCount, double multiplier) {
        double xyStdDev = multiplier
                * (0.1)
                * ((0.01 * Math.pow(lowestDist, 2.0)) + (0.005 * Math.pow(avgDist, 2.0)))
                / tagCount;
        return Math.max(0.02, xyStdDev);
    }

//    private void logVisionData(List<Pose3d> tagPoses, double xyStdDev, Pose2d camera_pose, double timestamp) {
//        LogUtil.recordPose3d("Vision " + mConstants.kTableName + "/Tag Poses", tagPoses.toArray(new Pose3d[0]));
//        SmartDashboard.putNumber("Vision " + mConstants.kTableName + "/N Tags Seen", tagPoses.size());
//        SmartDashboard.putNumber("Vision " + mConstants.kTableName + "/Calculated STDev", xyStdDev);
//        LogUtil.recordPose2d("Vision " + mConstants.kTableName + "/Camera Pose", camera_pose);
//        LogUtil.recordPose2d(
//                "Vision " + mConstants.kTableName + "/Robot Pose",
//                camera_pose.transformBy(mConstants.kRobotToCamera));
//        LogUtil.recordPose2d(
//                "Vision " + mConstants.kTableName + "/Relevant Odometry Pose",
//                RobotState.getInstance().getFieldToVehicle(timestamp));
//    }

    private void updateRobotState(Swerve swerve, double timestamp, Pose2d pose, double xyStdDev) {

    }

//    private double calculateRotation(Pose2d camera_pose, boolean isRedAlliance) {
//        double rotationDegrees = camera_pose
//                .transformBy(mConstants.kRobotToCamera)
//                .getRotation()
//                .getDegrees()
//                + 180.0;
//
//        if (!isRedAlliance) {
//            return Util.boundAngleNeg180to180Degrees(rotationDegrees);
//        } else {
//            return Util.boundAngle0to360Degrees(rotationDegrees);
//        }
//    }

//    private void logRotation(double rotationDegrees) {
//        SmartDashboard.putNumber("Vision Heading/" + mConstants.kTableName, rotationDegrees);
//        VisionDeviceManager.getInstance().getMovingAverage().addNumber(rotationDegrees);
//    }

    /**
     * Get the current pose estimate from the Photon Camera
     * @return The current pose estimate
     */
    public Optional<EstimatedRobotPose> getPosEstimate() {
        return photonPoseEstimator.update();
    }

    /**
     * Get the Photon Camera object
     * @return The Photon Camera object
     */
    public PhotonCamera returnCam() {
        return photonCamera;
    }

    /**
     * Get the Photon Pose Estimator object
     * @return The Photon Pose Estimator object
     */
    public PhotonPoseEstimator returnPoseEstimator() {
        return photonPoseEstimator;
    }



}
