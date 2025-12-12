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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  // Simulator
  private ElevatorSim sim =
      new ElevatorSim(simV, simA, gearbox, minPosition_m, maxPosition_m, true, 0.0, 0.0, 0.0);

  // Motion profiling
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(simS, simG, simV, simA, 0.02);
  private final PIDController pid = new PIDController(simP, simI, simD);

  // Variables
  private double motorVoltage = 0.0;
  private double lastNextPositionMeters;

  @Override
  public void updateInputs(ElevatorIOInputs ioInputs) {
    // Update simulation inputs
    sim.setInput(motorVoltage);
    sim.update(0.02);

    ioInputs.rotorAngle_rot = sim.getPositionMeters() * motorRatio_rotpm;
    ioInputs.rotorVelocity_rotps = sim.getVelocityMetersPerSecond() * motorRatio_rotpm;
    ioInputs.mechanismPosition_m = sim.getPositionMeters();
    ioInputs.mechanismVelocity_mps = sim.getVelocityMetersPerSecond();

    ioInputs.motorVoltage_V = motorVoltage;
    ioInputs.statorCurrent_A = sim.getCurrentDrawAmps();
    ioInputs.supplyCurrent_A = ioInputs.statorCurrent_A * (motorVoltage / 12.0);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motorVoltage = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
    lastNextPositionMeters = sim.getPositionMeters();
  }

  @Override
  public void setNextState(Distance nextHeight, LinearVelocity nextVelocity) {
    motorVoltage =
        MathUtil.clamp(
            feedforward.calculateWithVelocities(
                    sim.getVelocityMetersPerSecond(), nextVelocity.in(MetersPerSecond))
                + pid.calculate(sim.getPositionMeters(), lastNextPositionMeters),
            -12.0,
            12.0);
    lastNextPositionMeters = nextHeight.in(Meters);
  }
}
