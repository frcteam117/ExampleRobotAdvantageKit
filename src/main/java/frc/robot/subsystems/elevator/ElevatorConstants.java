// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorConstants {
  public static final String name = "Elevator";

  // Mechanism limits
  public static final double maxVelocity_mps = 2;
  public static final double maxAcceleration_mps2 = 6;
  public static final double maxPosition_m = 3;
  public static final double minPosition_m = 0;

  // Mechanism 2d config
  public static final double mechanismWidthMax = 1.7;
  public static final double mechanismHeightMax = 3.35;

  // Device CAN IDs
  public static final int LeaderCanId = 10;
  public static final int FollowerCanId = 11;

  // Motor configuration
  public static final double motorRatio_rotpm = 8;
  public static final int currentLimit = 15;
  public static final boolean leftInverted = false;
  public static final boolean rightInverted = true;
  public static final DCMotor gearbox = DCMotor.getNEO(2);
  public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

  // Velocity PID configuration
  public static final double realP = 0.0;
  public static final double realI = 0.0;
  public static final double realD = 0.0;
  public static final double realS = 0.0;
  public static final double realV = 0.1;
  public static final double realA = 0.227;
  public static final double realG = 0;

  public static final double simP = 1;
  public static final double simI = 0.0;
  public static final double simD = 0.0;
  public static final double simS = 0.0;
  public static final double simV = 0.05;
  public static final double simA = 0.001;
  public static final double simG = 0.0098;

  static {
    motorConfig.smartCurrentLimit(20, 40).voltageCompensation(12.0);
    motorConfig.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0 / 60.0);
  }
}
