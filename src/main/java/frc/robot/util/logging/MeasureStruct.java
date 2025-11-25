// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging;

import edu.wpi.first.units.ImmutableMeasure;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class MeasureStruct<U extends Unit> implements Struct<Measure<U>> {
  private final U unit;

  public MeasureStruct(U unit) {
    this.unit = unit;
  }

  @SuppressWarnings("unchecked")
  @Override
  public Class<Measure<U>> getTypeClass() {
    return (Class<Measure<U>>) ((Measure<U>) new ImmutableMeasure<U>(1, 1, unit)).getClass();
  }

  @Override
  public String getTypeName() {
    return LogUtil.toSuffix(unit.symbol()).substring(1);
  }

  @Override
  public int getSize() {
    return kSizeDouble;
  }

  @Override
  public String getSchema() {
    return "double magnitude" + LogUtil.toSuffix(unit.symbol());
  }

  @SuppressWarnings("unchecked")
  @Override
  public Measure<U> unpack(ByteBuffer bb) {
    return (Measure<U>) unit.of(bb.getDouble());
  }

  @Override
  public void pack(ByteBuffer bb, Measure<U> value) {
    bb.putDouble(value.in(unit));
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
