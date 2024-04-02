// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class PointToAngleDrive extends Command {
  private final PIDController pointController;
  private final Drive drive;
  private final LoggedTunableNumber pointKp;
  private final LoggedTunableNumber pointKd;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;

  public PointToAngleDrive(
      Drive drive,
      LoggedTunableNumber pointKp,
      LoggedTunableNumber pointKd,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier desiredAngleSupplier) {
    this.drive = drive;
    this.pointKp = pointKp;
    this.pointKd = pointKd;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    pointController = new PIDController(pointKp.get(), 0.0, pointKd.get());
    omegaSupplier =
        () -> {
          double currentRotation =
              MathUtil.inputModulus(drive.getRotation().getRadians(), 0, Math.PI * 2);
          double desiredAngle = desiredAngleSupplier.getAsDouble();
          double output = pointController.calculate(currentRotation, desiredAngle);
          output = MathUtil.clamp(output, -1, 1);
          Logger.recordOutput(
              "Drive/PointAt/AngleCurrent",
              new Pose2d(
                  new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  new Rotation2d(currentRotation)));
          Logger.recordOutput(
              "Drive/PointAt/AngleDesired",
              new Pose2d(
                  new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  new Rotation2d(desiredAngle)));
          return output;
        };

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pointController.setPID(pointKp.get(), 0.0, pointKd.get());
    pointController.enableContinuousInput(0, Math.PI * 2);
  }

  @Override
  public void execute() {
    if (pointKp.hasChanged(hashCode()) || pointKd.hasChanged(hashCode())) {
      pointController.setPID(pointKp.get(), 0.0, pointKd.get());
    }
    drive.runVelocity(drive.createChassisSpeeds(xSupplier, ySupplier, omegaSupplier, false));
  }

  @Override
  public void end(boolean interrupted) {
    pointController.close();
  }

  public boolean isFacingDesiredAngle() {
    return pointController.atSetpoint();
  }
}
