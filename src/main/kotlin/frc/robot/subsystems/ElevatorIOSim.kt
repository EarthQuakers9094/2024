package frc.robot.subsystems


class ElevatorIOSparkMax(): ElevatorIO() {
    private val liftSparkMax = CANSparkMax(liftMotorId,CANSparkLowLevel.MotorType.kBrushless);

    private val desiredPosition = 0.1;

    private val pidController = PIDController(
                    Constants.Elevator.sim_pid.kP,
                    Constants.Elevator.sim_pid.kI,
                    Constants.Elevator.sim_pid.kD
            );

     private var elevator =
            ElevatorSim(
                    DCMotor.getNEO(2),
                    Constants.Elevator.gearing,
                    10.0,
                    0.2,
                    0.1,
                    0.762,
                    true,
                    0.762
            );

    init {}

    fun setDesiredLocation(location: Double) {
        desiredPosition = location;
    }

    fun updateInputs(inputs: ElevatorIOInputs) {
        val res =
                min(1.0, max(pidController.calculate(elevator.positionMeters, desiredPosition), -1.0)) +
                        Constants.Elevator.sim_feedforward
        SmartDashboard.putNumber("pid output", res)

        elevator.setInputVoltage(res * RobotController.getInputVoltage())

        elevator.update(0.02)

        inputs.height = elevator.positionMeters    
    }
}
