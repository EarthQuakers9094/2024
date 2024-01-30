package frc.robot.subsystems


class IntakeIOSparkMax(private val intakeCANId: Int, private val limitSwitchId: Int) {

    private val intakeSparkMax = CANSparkMax(intakeCANId, CANSparkLowLevel.MotorType.kBrushless)

    private val limitSwitch = DigitalInput(limitSwitchId)

    init {
        intakeSparkMax.restoreFactoryDefaults()
    }

    fun setSpeed(speed: Double) {
        intakeSparkMax.set(speed);
    }

    fun updateInputs(inputs: ElevatorIOInputs) {
        inputs.limitSwitch = limitSwitch.get();
    }
}
