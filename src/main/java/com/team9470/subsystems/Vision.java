package com.team9470.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Vision {

    private static final List<Vision> cameras = new ArrayList<>();
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


    public Vision(String name, Transform3d amongus) {
        this.photonCamera = new PhotonCamera(name);
        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                photonCamera,
                amongus// Robot to camera transform (adjust as needed)

        );
        cameras.add(this);

    }
    //Return Robot Estimated Pos
    public Optional<EstimatedRobotPose> getPosEstimate() {
        return photonPoseEstimator.update();
    }
    public PhotonCamera returnCam() {
        return photonCamera;
    }
    public static List<EstimatedRobotPose> getPoses() {
        List<EstimatedRobotPose> poses = new ArrayList<>();
        for(Vision c: cameras){
            if(c.getPosEstimate().isPresent()) poses.add(c.getPosEstimate().get());
        }
        return poses;
    }


    private static boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

}
