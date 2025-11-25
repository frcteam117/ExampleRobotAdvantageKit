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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.motionProfiling.ProfileController.ProfileState;

public class ElevatorIOSpark implements ElevatorIO {
  // Sparkmax objects
  private SparkMax leadingSpark = new SparkMax(LeaderCanId, MotorType.kBrushless);
  private SparkMax followingSpark = new SparkMax(FollowerCanId, MotorType.kBrushless);
  private RelativeEncoder encoder = leadingSpark.getEncoder();
  private SparkMaxSim sim = new SparkMaxSim(leadingSpark, gearbox);

  // Motion profiling
  private ElevatorFeedforward feedforward =
      new ElevatorFeedforward(realS, realG, realV, realA, 0.02);
  private PIDController pid = new PIDController(realP, realI, realD);

  public ElevatorIOSpark() {
    leadingSpark.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingSpark.configure(
        motorConfig.follow(leadingSpark, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateState(ElevatorIOState ioState) {
    // ioState.mechanismHeight = Meters.of(sim.getPositionMeters());
    // ioState.mechanismVelocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());
    // ioState.motorVoltage = Volts.of(motorVoltage);
    // ioState.statorCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leadingSpark.setVoltage(MathUtil.clamp(voltage.in(Volts), -12.0, 12.0));
  }

  @Override
  public void setNextState(ProfileState<DistanceUnit, LinearVelocityUnit> nextState) {
    leadingSpark.setVoltage(
        MathUtil.clamp(
            feedforward.calculateWithVelocities(
                    encoder.getVelocity(), nextState.nextVelocity.in(MetersPerSecond))
                + pid.calculate(encoder.getPosition(), nextState.lastNextPosition.in(Meters)),
            -12.0,
            12.0));
  }
}
