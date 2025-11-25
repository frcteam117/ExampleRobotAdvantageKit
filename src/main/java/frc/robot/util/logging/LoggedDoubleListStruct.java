// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class LoggedDoubleListStruct implements Struct<Double[]> {
  private final int length;
  private final String schema;

  public LoggedDoubleListStruct(String... names) {
    length = names.length;
    String tempSchema = "double ";
    for (int i = 0; i < length; i++) {
      tempSchema += names[i] + ";double ";
    }
    schema = tempSchema.substring(0, tempSchema.length() - 8);
  }

  @Override
  public Class<Double[]> getTypeClass() {
    return Double[].class;
  }

  @Override
  public String getTypeName() {
    return "DoubleList";
  }

  @Override
  public int getSize() {
    return kSizeDouble * length;
  }

  @Override
  public String getSchema() {
    return schema;
  }

  @Override
  public Double[] unpack(ByteBuffer bb) {
    Double[] list = new Double[length];
    for (int i = 0; i < length; i++) {
      list[i] = bb.getDouble();
    }
    return list;
  }

  @Override
  public void pack(ByteBuffer bb, Double[] value) {
    for (int i = 0; i < length; i++) {
      bb.putDouble(value[i]);
    }
  }

  @Override
  public boolean isImmutable() {
    return false;
  }
}
