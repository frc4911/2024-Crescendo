package com.cyberknights4911.drive;

import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public final class PointToPointDrive extends Command {
  private static final LoggedTunableNumber pointKp = new LoggedTunableNumber("Drive/GoTo/kP", 0.5);
  private static final LoggedTunableNumber pointKd = new LoggedTunableNumber("Drive/GoTo/kD", 0.0);

  private final PIDController pointController;
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  static PointToPointDrive createDriveToPoint(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double targetX,
      double targetY) {
    return new PointToPointDrive(drive, xSupplier, ySupplier, targetX, targetY);
  }

  private PointToPointDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double targetX,
      double targetY) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    pointController = new PIDController(pointKp.get(), 0.0, pointKd.get());
    pointController.setTolerance(0.1);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pointController.setPID(pointKp.get(), 0.0, pointKd.get());
  }

  @Override
  public void execute() {
    double currentX = drive.getPose().getTranslation().getX();
    double currentY = drive.getPose().getTranslation().getY();

    double outputX = pointController.calculate(currentX, xSupplier.getAsDouble());
    double outputY = pointController.calculate(currentY, ySupplier.getAsDouble());

    drive.runVelocity(new ChassisSpeeds(outputX, outputY, 0));
}

  @Override
  public void end(boolean interrupted) {

}

  public boolean isAtTarget() {
    return pointController.atSetpoint();
  }
}