package com.cyberknights4911.constants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.HashMap;
import java.util.Map;

@RecordBuilder
public record SwerveConstants(
  double maxLinearSpeed,
  double trackWidthX,
  double trackWidthY,
  double wheelDiameter,
  double treadWear,
  double updateFrequency,
  double statusTimeout,
  boolean angleInverted,
  boolean driveInverted,
  NeutralModeValue angleNeutralMode,
  NeutralModeValue driveNeutralMode,
  double driveOpenRamp,
  double driveClosedRamp,
  PidValues anglePidValues,
  PidValues drivePidValues,
  double angleCurrentLimit,
  double driveCurrentLimit,
  SteerGearRatio steerGearRatio,
  DriveGearRatio driveGearRatio,
  SwerveModuleConstants frontLeft,
  SwerveModuleConstants frontRight,
  SwerveModuleConstants backLeft,
  SwerveModuleConstants backRight
) {

  public static SwerveConstants get() {
    SwerveConstants constants = ROBOTS.get(Constants.get().name());
    if (constants == null) {
      throw new IllegalStateException(
        "No SwerveConstants defined for robot named: " + Constants.get().name());
    }
    return constants;
  }

  private static Map<String, SwerveConstants> ROBOTS = new HashMap<>();
  static {
    ROBOTS.put(Constants.WHAM.name(), getWhamConstants());
    // TODO: put others here too
  }

  private static SwerveConstants getWhamConstants() {
    return SwerveConstantsBuilder.builder()
        .maxLinearSpeed(4.5)
        .trackWidthX(Units.inchesToMeters(22.75))
        .trackWidthY(Units.inchesToMeters(22.75))
        .wheelDiameter(Units.inchesToMeters(4))
        .treadWear(0)
        .updateFrequency(0.02)
        .angleInverted(true)
        .driveInverted(false)
        .angleNeutralMode(NeutralModeValue.Coast)
        .driveNeutralMode(NeutralModeValue.Brake)
        .steerGearRatio(SteerGearRatio.MK4i)
        .driveGearRatio(DriveGearRatio.L1)
        .anglePidValues(new PidValues(0, 0, 0, 0, 0, 0))
        .drivePidValues(new PidValues(0, 0, 0, 0, 0, 0))
        .angleCurrentLimit(25)
        .driveCurrentLimit(60)
        .frontLeft(
          SwerveModuleConstantsBuilder.builder()
            .name("FrontLeft")
            .driveMotorId(2)
            .steerMotorId(6)
            .canCoderId(1)
            .absoluteSteerOffset(21)
            .build()
        )
        .frontRight(
          SwerveModuleConstantsBuilder.builder()
            .name("FrontRight")
            .driveMotorId(1)
            .steerMotorId(5)
            .canCoderId(0)
            .absoluteSteerOffset(158)
            .build()
        )
        .backLeft(
          SwerveModuleConstantsBuilder.builder()
            .name("BackLeft")
            .driveMotorId(3)
            .steerMotorId(7)
            .canCoderId(2)
            .absoluteSteerOffset(193)
            .build()
        )
        .backRight(
          SwerveModuleConstantsBuilder.builder()
            .name("BackRight")
            .driveMotorId(4)
            .steerMotorId(8)
            .canCoderId(3)
            .absoluteSteerOffset(104)
            .build()
        )
        .build();
  }

  public static record PidValues(
    double p,
    double i,
    double d,
    double s,
    double v,
    double a) {}

  public enum DriveGearRatio {
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    L4((48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0));

    public final double reduction;

    DriveGearRatio(double reduction) {
      this.reduction = reduction;
    }
  }

  public enum SteerGearRatio {
    MK4i((32.0 / 15.0) * (60.0 / 10.0));

    public final double reduction;

    SteerGearRatio(double reduction) {
      this.reduction = reduction;
    }
  }
}