package frc.robot.util.logging;

import edu.wpi.first.util.struct.StructSerializable;
import org.littletonrobotics.junction.Logger;

public class LoggedDoubleList implements StructSerializable {
  public final String key;
  public final LoggedDoubleListStruct struct;
  public Double[] values;

  public LoggedDoubleList(String key, String... names) {
    this.key = key;
    struct = new LoggedDoubleListStruct(names);
    values = new Double[names.length];
  }

  public void setValues(Double... values) {
    this.values = values;
  }

  public void log() {
    Logger.recordOutput(key, struct, values);
  }
}
