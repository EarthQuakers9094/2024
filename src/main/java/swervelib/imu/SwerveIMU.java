package swervelib.imu;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

/**
 * Swerve IMU abstraction to define a standard interface with a swerve drive.
 */
public abstract class SwerveIMU
{
  @AutoLog
  public class SwerveIMUInput {
    public Rotation3d rawRotation3d = Rotation3d(0.0,0.0,0.0);
    public Rotation3d rotation3d = Rotation3d(0.0,0.0,0.0);
    public Optional<Translation3d> acceleration = Optional.of(Translation3d(0.0,0.0,0.0));
  }

  public SwerveIMUInputAutoLogged inputs;

  public void updateInputs() {
      inputs.rawRotation3d = getRawRotation3dInternal();
      inputs.rotation3d = getRotation3dInternal();
      inputs.acceleration = getAccelInternal();

      Logger.processInputs("Gyro", inputs)
  }

  /**
   * Reset IMU to factory default.
   */
  public default void factoryDefault() {}

  /**
   * Clear sticky faults on IMU.
   */
  public default void clearStickyFaults() {}

  /**
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public default void setOffset(Rotation3d offset) {}

  /**
   * Set the gyro to invert its default direction.
   *
   * @param invertIMU gyro direction
   */
  public default void setInverted(boolean invertIMU) {}

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  public default Rotation3d getRawRotation3dInternal() {
    return Rotation3d(0.0,0.0,0.0);
  }

  public default Rotation3d getRawRotation3d() {
    return inputs.rawRotation3d;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  public default Rotation3d getRotation3dInternal() {
    return Rotation3d(0.0,0.0,0.0);
  }

  public default Rotation3d getRotation3d() {
    return inputs.rotation3d;
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  public default Optional<Translation3d> getAccelInternal() {
    return Translation3d(0.0,0.0,0.0);
  }

  public default Optional<Translation3d> getAccel() {
    return inputs.acceleration;
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  public default Object getIMU() {
      return ();
  };
}
