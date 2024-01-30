package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.ControlType
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
class Elevator(private val io: ElevatorIO) : SubsystemBase() {

    private var desiredPosition = 0.2

    private var averagePostion = MovingAverage(10)

    private var inputs = ElevatorIO.ElevatorIOInputsAutoLogged();

    init {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs)

        SmartDashboard.putNumber("desired elevator location", 0.1);

        averagePostion.setAverage(inputs.height);
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs)

        SmartDashboard.putNumber("elevator position", liftSparkMax.encoder.position)
        desiredPosition = SmartDashboard.getNumber("desired elevator location", 0.1)
        averagePostion.addValue(inputs.height)
    }

    fun setPosition(position: Double) {
        desiredPosition = position
        io.setDesiredLocation(position)
    }

    fun atLocation(): Boolean {
        return abs(averagePostion.getAverage() - desiredPosition) <= 0.05
    }
}
