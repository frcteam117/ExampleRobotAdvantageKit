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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
            new ElevatorSim(
                    simV,
                    simA,
                    gearbox,
                    minPosition.in(Meters),
                    maxPosition.in(Meters),
                    true,
                    0.0,
                    0.0,
                    0.0);

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

        ioInputs.rotorAngle = Rotations.of(sim.getPositionMeters() * rotationsperMeter);
        ioInputs.rotorVelocity =
                RotationsPerSecond.of(sim.getVelocityMetersPerSecond() * rotationsperMeter);
        ioInputs.mechanismHeight = Meters.of(sim.getPositionMeters());
        ioInputs.mechanismVelocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());

        ioInputs.motorVoltage = Volts.of(motorVoltage);
        ioInputs.statorCurrent = Amps.of(sim.getCurrentDrawAmps());
        ioInputs.supplyCurrent = ioInputs.statorCurrent.times(motorVoltage / 12.0);
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
                                        sim.getVelocityMetersPerSecond(),
                                        nextVelocity.in(MetersPerSecond))
                                + pid.calculate(sim.getPositionMeters(), lastNextPositionMeters),
                        -12.0,
                        12.0);
        lastNextPositionMeters = nextHeight.in(Meters);
    }
}
