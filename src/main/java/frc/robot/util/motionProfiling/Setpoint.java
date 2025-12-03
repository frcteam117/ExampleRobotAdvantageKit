package frc.robot.util.motionProfiling;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.StructSerializable;

public record Setpoint(Distance m, LinearVelocity ms, Voltage vt) implements StructSerializable {}
