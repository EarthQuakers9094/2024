package frc.robot.subsystems


class IntakeIO() {
    @AutoLog
    public class IntakeIOInputs {
        public limitSwitch = false;
    }

    fun setSpeed(current: Double) {}

    fun updateInputs(inputs: ElevatorIOInputs) {}
}
