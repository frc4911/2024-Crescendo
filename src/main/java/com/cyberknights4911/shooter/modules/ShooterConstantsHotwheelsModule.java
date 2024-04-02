// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.shooter.modules;

import com.cyberknights4911.shooter.ShooterConstants;
import com.cyberknights4911.shooter.ShooterConstantsBuilder;
import com.cyberknights4911.util.FeedForwardValues;
import com.cyberknights4911.util.PidValues;
import dagger.Module;
import dagger.Provides;

@Module
public interface ShooterConstantsHotwheelsModule {

  @Provides
  public static ShooterConstants providesShooterConstants() {
    return ShooterConstantsBuilder.builder()
        .shooterMotorTopId(21)
        .shooterMotorBottomId(22)
        .aimerMotorId(23)
        .guideMotorId(24)
        .sensorId(2)
        .aimerGearRatio((60 / 20) * (60 / 20))
        .guidePercentOutput(.2)
        .guideReversePercentOutput(-0.05)
        .feedTime(2)
        .aimTime(.5)
        .speakerPositionDegrees(55)
        .podiumPositionDegrees(38) // was 34, did not try 36, trying 38
        .collectPositionDegrees(34)
        .firePercentOutput(.6)
        .shooterFeedBackValues(new PidValues(0.1, 0, 0))
        .shooterFeedForwardValues(new FeedForwardValues(0, 0))
        .aimerFeedBackValues(new PidValues(0.3, 0, 0))
        .aimerFeedForwardValues(new FeedForwardValues(0, 0))
        .guideFeedBackValues(new PidValues(0.1, 0, 0))
        .guideFeedForwardValues(new FeedForwardValues(0, 0))
        .beamThreshold(.1)
        .build();
  }
}
