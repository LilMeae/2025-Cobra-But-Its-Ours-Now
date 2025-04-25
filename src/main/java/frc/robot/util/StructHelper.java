package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * Struct helper simplfies publishing structs for NetworkTables
 * @author Alexander Szura team 5409
 */
public final class StructHelper {

  private static final ArrayList<Pair<StructPublisher<?>, Supplier<?>>> VALUES = new ArrayList<>();

  private StructHelper() {}

  /**
   * Publishs and updates a Struct object to NT ({@link NetworkTableInstance#getDefault()}).
   *
   * @param <TYPE> The type of struct to publish
   * @param name The name to publish
   * @param struct The Struct to publish
   * @param value The value of the struct
   */
  public static final <TYPE> void publishStruct(String name, Struct<TYPE> struct, Supplier<TYPE> value) {
    StructPublisher<TYPE> publisher = NetworkTableInstance.getDefault()
        .getStructTopic(name, struct)
        .publish();

    VALUES.add(new Pair<>(publisher, value));
  }

  /**
   * Publishs and updates a Struct object to Shuffleboard.
   *
   * @param <TYPE> The type of struct to publish
   * @param tabName The location on shuffleboard to put the struct
   * @param name The name to publish
   * @param struct The Struct to publish
   * @param value The value of the struct
   */
  public static final <TYPE> void publishStruct(String tabName, String name, Struct<TYPE> struct, Supplier<TYPE> value) {
    StructPublisher<TYPE> publisher = NetworkTableInstance.getDefault()
        .getTable("Shuffleboard")
        .getSubTable(tabName)
        .getStructTopic(name, struct)
        .publish();

    VALUES.add(new Pair<>(publisher, value));
  }

  /**
   * This method should only be called periodically ONCE. Do not call this method if you don't know
   * what you are doing
   */
  public static final void update() {
    for (int i = 0; i < VALUES.size(); i++) {
      updatePair(VALUES.get(i));
    }
  }

  @SuppressWarnings("unchecked")
  private static final <T> void updatePair(Pair<StructPublisher<?>, Supplier<?>> pair) {
    StructPublisher<T> publisher = (StructPublisher<T>) pair.getFirst();
    Supplier<T> supplier = (Supplier<T>) pair.getSecond();
    publisher.set(supplier.get());
  }
}
