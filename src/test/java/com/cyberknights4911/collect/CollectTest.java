// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.collect;

import com.cyberknights4911.logging.LoggedTunableNumberFactory;
import com.cyberknights4911.util.PidValues;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class CollectTest {
  private CollectConstants constants;
  @Mock private CollectIO collectIO;
  @Mock private LoggedTunableNumberFactory numberFactory;

  @BeforeEach
  public void setup() {
    constants =
        CollectConstantsBuilder.builder()
            .motorCollectRightId(1)
            .motorCollectLeftId(2)
            .collectGearRatio(1.0)
            .collectFeedBackValues(new PidValues(0, 0, 0))
            .build();
  }

  @Test
  public void constructor() {
    // Collect collect = new Collect(constants, collectIO, numberFactory);
  }
}
