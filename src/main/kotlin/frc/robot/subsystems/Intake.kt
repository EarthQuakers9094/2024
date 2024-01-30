package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

/** Creates a new ExampleSubsystem. */
class Intake(private val io: IntakeIO) : SubsystemBase() {

    private enum class State {
        Looking,
        // Idle,
        Holding
    }
    
    private var input = IntakeIO.IntakeIOInputsAutoLogged();

    private var state = State.Looking

    init {
        io.updateInputs(inputs);
        Logger.processInputs("Intake",inputs);
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake",inputs);
    
        when (state) {
            // State.Idle -> {
            //     if (!input.limitSwitch) {
            //         state = State.Looking
            //     }
            //     io.setSpeed(0.0)
            // }
            State.Looking -> {
                io.setSpeed(Constants.Intake.speed);

                if (inputs.limitSwitch) {
                    state = State.Holding
                }
            }
            State.Holding -> {
                io.setSpeed(0.0);

                if (!inputs.limitSwitch) {
                    state = State.Looking
                }
            }
        }
    }
}
