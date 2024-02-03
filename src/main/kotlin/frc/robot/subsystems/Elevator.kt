package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.utils.MovingAverage
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/** Creates a new ExampleSubsystem. */
class Elevator(private val liftMotorId: Int) : SubsystemBase() {

    private val liftSparkMax = CANSparkMax(liftMotorId, CANSparkLowLevel.MotorType.kBrushless)

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
            )

    private var sim_pid =
            PIDController(
                    Constants.Elevator.sim_pid.kP,
                    Constants.Elevator.sim_pid.kI,
                    Constants.Elevator.sim_pid.kD
            )

    private var desiredPosition = 0.2

    private var averagePostion = MovingAverage(10)

    init {
        liftSparkMax.restoreFactoryDefaults()

        liftSparkMax.encoder.positionConversionFactor = 1.0

        liftSparkMax.pidController.p = Constants.Elevator.pid.kP
        liftSparkMax.pidController.i = Constants.Elevator.pid.kI
        liftSparkMax.pidController.d = Constants.Elevator.pid.kD

        SmartDashboard.putData("sim pid", sim_pid)
        SmartDashboard.putNumber("desired elevator location", 0.1)

        if (!RobotBase.isReal()) {
            liftSparkMax.encoder.position = elevator.positionMeters
        }

        averagePostion.setAverage(liftSparkMax.encoder.position)
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        SmartDashboard.putNumber("elevator position", liftSparkMax.encoder.position)
        desiredPosition = SmartDashboard.getNumber("desired elevator location", 0.1)
        averagePostion.addValue(liftSparkMax.encoder.position)
    }

    fun setPosition(position: Double) {
        desiredPosition = position
        liftSparkMax.pidController.setReference(position, CANSparkBase.ControlType.kPosition)
    }

    fun atLocation(): Boolean {
        return abs(averagePostion.getAverage() - desiredPosition) <= 0.05
    }

    override fun simulationPeriodic() {
        val res =
                min(1.0, max(sim_pid.calculate(elevator.positionMeters, desiredPosition), -1.0)) +
                        Constants.Elevator.sim_feedforward
        SmartDashboard.putNumber("pid output", res)

        elevator.setInputVoltage(res * RobotController.getInputVoltage())

        elevator.update(0.02)

        liftSparkMax.encoder.position = elevator.positionMeters
    }
}
