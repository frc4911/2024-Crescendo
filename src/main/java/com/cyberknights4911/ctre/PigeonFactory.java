package com.cyberknights4911.ctre;

import com.ctre.phoenix.sensors.BasePigeon;

/** Factory for creating BasePigeon objects. */
public interface PigeonFactory {
  BasePigeon create(int id);
}
