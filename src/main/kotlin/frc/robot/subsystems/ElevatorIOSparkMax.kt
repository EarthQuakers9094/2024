package frc.robot.subsystems


class ElevatorIOSparkMax(private val liftMotorId: Int): ElevatorIO() {
    private val liftSparkMax = CANSparkMax(liftMotorId,CANSparkLowLevel.MotorType.kBrushless);
    
    init {
        liftSparkMax.restoreFactoryDefaults();
        liftSparkMax.encoder.positionConversionFactor = 1.0;

        liftSparkMax.pidController.p = Constants.Elevator.pid.kP
        liftSparkMax.pidController.i = Constants.Elevator.pid.kI
        liftSparkMax.pidController.d = Constants.Elevator.pid.kD

    }

    fun setDesiredLocation(location: Double) {
        liftSparkMax.pidController.setReference(location, CANSparkBase.ControlType.kPosition);
    }

    fun updateInputs(inputs: ElevatorIOInputs) {
        inputs.height = liftSparkMax.encoder.position;
    }
}
