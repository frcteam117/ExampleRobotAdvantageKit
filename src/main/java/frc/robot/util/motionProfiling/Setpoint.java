package frc.robot.util.motionProfiling;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.util.logging.RecordStruct;

public record Setpoint(Distance m, LinearVelocity ms, Voltage vt) implements StructSerializable {
  @SuppressWarnings("unchecked")
  public static Struct<Setpoint> struct = new RecordStruct(Setpoint.class, "Setpoint");
}
