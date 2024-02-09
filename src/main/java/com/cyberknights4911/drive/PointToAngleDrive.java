// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class PointToAngleDrive extends Command {
  private static final LoggedTunableNumber pointKp = new LoggedTunableNumber("Drive/Point/kP", 5.0);
  private static final LoggedTunableNumber pointKd = new LoggedTunableNumber("Drive/Point/kD", 0.0);

  private final PIDController pointController;
  private final Drive drive;
  private final ControlConstants controlConstants;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;

  static PointToAngleDrive createDriveFacingPoint(
      Drive drive,
      ControlConstants controlConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double x,
      double y) {

    DoubleSupplier angleSupplier =
        () -> {
          double currentX = drive.getPose().getTranslation().getX();
          double currentY = drive.getPose().getTranslation().getY();
          // TODO calculate angle based on current position and desired point
          // Must use trig!!!!!!!!!!!!
          return 0.0;
        };
    return new PointToAngleDrive(drive, controlConstants, xSupplier, ySupplier, angleSupplier);
  }

  static PointToAngleDrive createDriveFacingFixedAngle(
      Drive drive,
      ControlConstants controlConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double angleRadians) {
    return new PointToAngleDrive(drive, controlConstants, xSupplier, ySupplier, () -> angleRadians);
  }

  private PointToAngleDrive(
      Drive drive,
      ControlConstants controlConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier desiredAngleSupplier) {
    this.drive = drive;
    this.controlConstants = controlConstants;
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
          Logger.recordOutput("Drive/PointAt/Current", currentRotation);
          Logger.recordOutput("Drive/PointAt/Desired", desiredAngle);
          Logger.recordOutput("Drive/PointAt/Output", output);
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
    drive.runVelocity(
        drive.createChassisSpeeds(controlConstants, xSupplier, ySupplier, omegaSupplier));
  }

  @Override
  public void end(boolean interrupted) {
    pointController.close();
  }

  public boolean isFacingDesiredAngle() {
    return pointController.atSetpoint();
  }
}
