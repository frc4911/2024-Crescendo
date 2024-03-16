// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<VisionUpdate> visionConsumer;
  private final CameraConfig[] cameraConfigs;

  private final HashMap<String, Double> lastTimestamp = new HashMap<>();

  private boolean updatePoseWithVisionReadings = true;
  private boolean useMaxValidDistanceAway = true;

  private final VisionConstants visionConstants;

  private static double MAX_ALLOWABLE_PITCH = 3;
  private static double MAX_ALLOWABLE_ROLL = 3;

  private static final LoggedTunableNumber twoTargetDeviation =
      new LoggedTunableNumber("Vision/deviations/twoTarget", 0.6);
  private static final LoggedTunableNumber threeTargetDeviation =
      new LoggedTunableNumber("Vision/deviations/threeTarget", 0.4);
  private static final LoggedTunableNumber fourTargetDeviation =
      new LoggedTunableNumber("Vision/deviations/fourTarget", 0.2);

  private static final Matrix<N3, N1> defaultDeviation = VecBuilder.fill(0.9, 0.9, 0.9);

  public Vision(
      VisionConstants visionConstants,
      Supplier<Pose2d> poseSupplier,
      Consumer<VisionUpdate> visionConsumer,
      CameraConfig... cameraConfigs) {
    super();
    this.visionConstants = visionConstants;
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
    this.cameraConfigs = cameraConfigs;

    Logger.recordOutput("Vision/updatePoseWithVisionReadings", true);
    Logger.recordOutput("Vision/useMaxValidDistanceAway", true);

    for (AprilTag tag : visionConstants.layout().getTags()) {
      Logger.recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }

    for (int i = 0; i < cameraConfigs.length; i++) {
      Logger.recordOutput(
          "Vision/" + cameraConfigs[i].constants().name(),
          new Pose3d().transformBy(cameraConfigs[i].constants().robotToCamera()));
      lastTimestamp.put(cameraConfigs[i].constants().name(), 0.0);
    }

    // TODO(rbrewer): warn for no cameras
    // NOTE: the camera object is not getting set in SIM or REPLAY
    // if (R_VisionIO.getCamera() == null) {
    //   System.out.println("NO RIGHT CAMERA");
    // }
    // if (L_VisionIO.getCamera() == null) {
    //   System.out.println("NO LEFT CAMERA");
    // }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameraConfigs.length; i++) {
      cameraConfigs[i].visionIO().updateInputs(cameraConfigs[i].inputs());
      Logger.processInputs(
          "Vision/" + cameraConfigs[i].constants().name(), cameraConfigs[i].inputs());
    }

    if (!updatePoseWithVisionReadings) {
      return;
    }

    // TODO(rbrewer): test whether necessary
    // if (Math.abs(drivetrain.getGyroPitch()) >= MAX_ALLOWABLE_PITCH
    //     || Math.abs(drivetrain.getGyroRoll()) >= MAX_ALLOWABLE_ROLL) {

    //   Logger.recordOutput("Vision/ValidGyroAngle", false);
    //   return;
    // }
    // Logger.recordOutput("Vision/ValidGyroAngle", true);

    for (int i = 0; i < cameraConfigs.length; i++) {
      updatePose(cameraConfigs[i]);
      Logger.recordOutput(
          "Vision/" + cameraConfigs[i].constants().name() + "Connected",
          cameraConfigs[i].inputs().lastTimestamp > 0.0);
    }
  }

  private void updatePose(CameraConfig cameraConfig) {
    boolean updated = false;
    boolean pnpFailed = false;
    Pose2d prevEstimatedRobotPose = poseSupplier.get();
    double timestamp = 0;
    double distance = 0.0;
    PhotonPipelineResult cameraResult = null;
    Pose3d robotPose = null;

    double standardDeviation = -1.0;

    // TODO(rbrewer): this is gross, find an alternative
    synchronized (cameraConfig.constants()) {
      // cameraResult = cameraConfig.inputs().lastResult;
      timestamp = cameraConfig.inputs().lastTimestamp;
    }

    // is this a new camera result?
    double prevTimestamp = lastTimestamp.get(cameraConfig.constants().name());
    if (prevTimestamp >= timestamp) {
      return;
    }
    lastTimestamp.put(cameraConfig.constants().name(), timestamp);

    int targetsSeen = cameraResult.getTargets().size();
    Logger.recordOutput("Vision/" + cameraConfig.constants().name() + "/SeenTargets", targetsSeen);

    if (targetsSeen > 1) {
      // more than one target seen, use PNP with PV estimator

      cameraConfig.visionIO().setReferencePose(prevEstimatedRobotPose);
      Optional<EstimatedRobotPose> result = cameraConfig.visionIO().update(cameraResult);

      if (result.isPresent()) {
        robotPose = result.get().estimatedPose;

        standardDeviation = getDeviationForNumTargets(targetsSeen);

        Logger.recordOutput(
            "Vision/" + cameraConfig.constants().name() + "/CameraPose",
            robotPose.transformBy(cameraConfig.constants().robotToCamera()));
      } else {
        pnpFailed = true;
      }

      Logger.recordOutput("Vision/" + cameraConfig.constants().name() + "/Ambiguity", 0.0);

    } else {
      // zero or 1 target, manually check if it is accurate enough
      for (PhotonTrackedTarget target : cameraResult.getTargets()) {
        if (isValidTarget(target)) {
          Transform3d cameraToTarget = target.getBestCameraToTarget();
          Optional<Pose3d> tagPoseOptional =
              visionConstants.layout().getTagPose(target.getFiducialId());
          if (tagPoseOptional.isEmpty()) {
            break;
          }

          Pose3d cameraPose = tagPoseOptional.get().transformBy(cameraToTarget.inverse());

          // TODO(rbrewer) verify this! naming was super sketch
          robotPose = cameraPose.transformBy(cameraConfig.constants().robotToCamera().inverse());

          standardDeviation = cameraToTarget.getTranslation().getNorm() / 2.0;

          Logger.recordOutput(
              "Vision/" + cameraConfig.constants().name() + "/distanceNorm3D",
              cameraToTarget.getTranslation().getNorm());

          Logger.recordOutput(
              "Vision/" + cameraConfig.constants().name() + "/Ambiguity",
              target.getPoseAmbiguity());
          Logger.recordOutput(
              "Vision/" + cameraConfig.constants().name() + "/CameraPose", cameraPose);
        }
      }
    }

    if (robotPose != null) {
      distance =
          prevEstimatedRobotPose
              .getTranslation()
              .getDistance(new Translation2d(robotPose.getX(), robotPose.getY()));

      Logger.recordOutput(
          "Vision/" + cameraConfig.constants().name() + "/DistanceFromRobot", distance);
      Logger.recordOutput(
          "Vision/" + cameraConfig.constants().name() + "/RobotPose", robotPose.toPose2d());
      Logger.recordOutput("Vision/" + cameraConfig.constants().name() + "/3DRobotPose", robotPose);

      // distance from vision estimate to last position estimate
      if (!useMaxValidDistanceAway
          || distance <= visionConstants.maxValidDistanceMeters() * targetsSeen) {
        // we passed all the checks, update the pose
        updated = true;

        Matrix<N3, N1> updateDeviation;
        // -1 = error when calculating deviation
        if (standardDeviation == -1) {
          updateDeviation = defaultDeviation;
        } else {
          updateDeviation = VecBuilder.fill(standardDeviation, standardDeviation, 0.9);
        }

        synchronized (visionConsumer) {
          visionConsumer.accept(new VisionUpdate(timestamp, robotPose.toPose2d(), updateDeviation));
        }
      }
    }

    Logger.recordOutput("Vision/" + cameraConfig.constants().name() + "/Updated", updated);
    Logger.recordOutput("Vision/" + cameraConfig.constants().name() + "/pnpFailed", pnpFailed);
    Logger.recordOutput(
        "Vision/" + cameraConfig.constants().name() + "/standardDeviation", standardDeviation);
  }

  public boolean tagVisible(int id, PhotonPipelineResult result) {
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == id && isValidTarget(target)) {
        return true;
      }
    }
    return false;
  }

  /**
   * returns the best Rotation3d from the robot to the given target.
   *
   * @param id
   * @return the Transform3d or null if there isn't
   */
  public Transform3d getTransform3dToTag(
      int id, PhotonPipelineResult result, Transform3d robotToCamera) {
    Transform3d bestTransform3d = null;

    for (PhotonTrackedTarget target : result.getTargets()) {
      // the target must be the same as the given target id, and the target must be available
      if (target.getFiducialId() != id || !isValidTarget(target)) {
        break;
      }

      bestTransform3d = robotToCamera.plus(target.getBestCameraToTarget());
    }
    return bestTransform3d;
  }

  public Rotation2d getAngleToTag(int id, PhotonPipelineResult result, Transform3d robotToCamera) {
    Transform3d transform = getTransform3dToTag(id, result, robotToCamera);
    if (transform != null) {
      return new Rotation2d(transform.getTranslation().getX(), transform.getTranslation().getY());
    } else {
      return null;
    }
  }

  public double getDistanceToTag(int id, PhotonPipelineResult result, Transform3d robotToCamera) {
    Transform3d transform = getTransform3dToTag(id, result, robotToCamera);
    if (transform != null) {
      return transform.getTranslation().toTranslation2d().getNorm();
    } else {
      return -1;
    }
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < visionConstants.maxAmbiguity()
        && visionConstants.layout().getTagPose(target.getFiducialId()).isPresent();
  }

  public void enableMaxDistanceAwayForTags() {
    Logger.recordOutput("Vision/useMaxValidDistanceAway", true);
    useMaxValidDistanceAway = true;
  }

  public void disableMaxDistanceAwayForTags() {
    Logger.recordOutput("Vision/useMaxValidDistanceAway", false);
    useMaxValidDistanceAway = false;
  }

  public void enableUpdatePoseWithVisionReadings() {
    Logger.recordOutput("Vision/updatePoseWithVisionReadings", true);
    updatePoseWithVisionReadings = true;
  }

  public void disableUpdatePoseWithVisionReadings() {
    Logger.recordOutput("Vision/updatePoseWithVisionReadings", false);
    updatePoseWithVisionReadings = false;
  }

  private double getDeviationForNumTargets(int numTargets) {
    if (numTargets == 2) {
      return twoTargetDeviation.get();
    }

    if (numTargets == 3) {
      return threeTargetDeviation.get();
    }

    if (numTargets > 3) {
      return fourTargetDeviation.get();
    }

    return 1.2;
  }
}

final class TargetComparator implements Comparator<PhotonTrackedTarget> {
  @Override
  public int compare(PhotonTrackedTarget a, PhotonTrackedTarget b) {
    double ambiguityDiff = a.getPoseAmbiguity() - b.getPoseAmbiguity();
    if (ambiguityDiff < 0.0) {
      return -1;
    }
    if (ambiguityDiff > 0.0) {
      return 1;
    }
    return 0;
  }
}
