package com.cyberknights4911.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.cyberknights4911.config.DeviceConfigurator;
import com.cyberknights4911.constants.SwerveConstants;
import com.cyberknights4911.ctre.PigeonFactory;
import com.cyberknights4911.logging.Alert;
import com.cyberknights4911.logging.Alert.AlertType;
import edu.wpi.first.math.util.Units;

public final class PigeonIO implements GyroIO, DeviceConfigurator {
  private static final int GYRO_INSTALL_YAW_ANGLE = 180;
  private final Alert pigeonConfigAlert = new Alert("Config for pigeon failed", AlertType.ERROR);
  private final Pigeon2 pigeon;

  public PigeonIO(PigeonFactory pigeonFactory) {
    System.out.println("[Init] Creating PigeonIO");
    pigeon = pigeonFactory.create(0);
    performConfig();
  }

  @Override
  public void performConfig() {
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.MountPoseYaw = GYRO_INSTALL_YAW_ANGLE;
    double timeout = SwerveConstants.get().statusTimeout();
    double frequency = SwerveConstants.get().updateFrequency();

    checkStatus(pigeon.getConfigurator().apply(config, timeout));
    checkStatus(pigeon.getPitch().setUpdateFrequency(frequency, timeout));
    checkStatus(pigeon.getRoll().setUpdateFrequency(frequency, timeout));
    checkStatus(pigeon.getYaw().setUpdateFrequency(frequency, timeout));
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.rollPositionRad = Units.degreesToRadians(pigeon.getRoll().getValue());
    inputs.pitchPositionRad = Units.degreesToRadians(pigeon.getPitch().getValue());
    inputs.yawPositionRad = Units.degreesToRadians(pigeon.getYaw().getValue());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityY().getValue());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityX().getValue());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZ().getValue());
  }

  private void checkStatus(StatusCode status) {
    if (status != StatusCode.OK) {
      pigeonConfigAlert.setText("Pigeon config failed: " + status.getDescription());
      pigeonConfigAlert.set(true);
    }
  }
}
