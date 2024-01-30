package swervelib.encoders;

/**
 * Swerve abstraction class to define a standard interface with absolute encoders for swerve modules..
 */
public interface SwerveAbsoluteEncoder
{

  @AutoLog
  public class SwerveAbsoluteEncoderInput {
    public double position = 0.0;
    public double velocity = 0.0;
  }

  public SwerveAbsoluteEncoderInputAutoLogged inputs;

  public final int moduleId;

  SwerveAbsoluteEncoder(int moduleNumber) {
    moduleId = moduleNumber;
  }

  public void updateInputs() {
      inputs.getAbsolutePositionInternal();
      inputs.getVelocityInternal();

      Logger.processInputs("SwerveModule/AbsoluteEncoder" + moduleId, inputs);
  }

  /**
   * The maximum amount of times the swerve encoder will attempt to configure itself if failures occur.
   */
  public final int     maximumRetries = 5;
  /**
   * Last angle reading was faulty.
   */
  public boolean readingError   = false;

  /**
   * Reset the encoder to factory defaults.
   */
  public default void factoryDefault() {}

  /**
   * Clear sticky faults on the encoder.
   */
  public default void clearStickyFaults() {}

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  public default void configure(boolean inverted) {}

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  public default double getAbsolutePositionInternal() {
    return 0.0;
  }

  public default double getAbsolutePosition() {
    return inputs.position;
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  public default Object getAbsoluteEncoder() {
    return ();
  }

  /**
   * Sets the Absolute Encoder offset at the Encoder Level.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  public default boolean setAbsoluteEncoderOffset(double offset) {}

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  public default double getVelocityInternal() {
      return 0.0;
  }

  public default double getVelocity() {
    return inputs.velocity;
  }
}
