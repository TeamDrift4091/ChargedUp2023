// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Links: 
 * https://docs.photonvision.org/en/latest/docs/examples/index.html
 * https://docs.photonvision.org/en/latest/docs/examples/apriltag.html
 * https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/apriltagExample/src/main/java/frc/robot/PhotonCameraWrapper.java
 * https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/apriltagExample/src/main/java/frc/robot/Drivetrain.java
 */
public class PhotonVisionWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    public PhotonVisionWrapper() {
        AprilTagFields field = AprilTagFields.k2023ChargedUp;
        AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(List.of(
            new AprilTag(1, new Pose3d())
        ), Units.feetToMeters(54), Units.feetToMeters(27));

        try {
            fieldLayout = new AprilTagFieldLayout(field.m_resourceFile);
        } catch (IOException e) {
            System.err.printf("Could not load resource file \"%s\".\n", field.m_resourceFile);
            e.printStackTrace();
        }

        photonCamera = new PhotonCamera(Constants.Vision.CAMERA_NAME);
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, Constants.Vision.CAMERA_TO_ROBOT);
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}