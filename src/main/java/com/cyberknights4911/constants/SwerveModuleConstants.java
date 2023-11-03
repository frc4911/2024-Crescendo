package com.cyberknights4911.constants;

import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record SwerveModuleConstants(
  String name,
  int driveMotorId,
  int steerMotorId,
  int canCoderId,
  int absoluteSteerOffset
) {}
