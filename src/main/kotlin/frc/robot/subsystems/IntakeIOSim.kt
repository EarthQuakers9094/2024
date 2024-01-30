package frc.robot.subsystems


class IntakeIOSim() {

    private val intakeSparkMax = CANSparkMax(intakeCANId, CANSparkLowLevel.MotorType.kBrushless)

    private val limitSwitch = DigitalInput(limitSwitchId)

    fun setSpeed(speed: Double) {
        SmartDashboard.putNumber("Intake/speed", speed);
    }

    fun updateInputs(inputs: ElevatorIOInputs) {
        inputs.limitSwitch = SmartDashboard.getBoolean("Intake/limitswitched");
    }
}
