package frc.robot.subsystems


class ElevatorIO() {
    @AutoLog
    public class ElevatorIOInputs {
        public height = 0.0;
    }

    fun setCurrent(current: Double) {}

    fun updateInputs(inputs: ElevatorIOInputs) {}
}
