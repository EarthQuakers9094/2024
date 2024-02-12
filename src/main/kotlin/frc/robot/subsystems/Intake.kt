package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants

/** Creates a new ExampleSubsystem. */
class Intake(private val intakeCANId: Int, private val intakeFollowCanId: Int
// private val limitSwitchId: Int
) : SubsystemBase() {

    private enum class State {
        Looking,
        // Idle,
        Holding
    }

    private val intakeSparkMax = CANSparkMax(intakeCANId, CANSparkLowLevel.MotorType.kBrushless)
    private val intakeFollowSparkMax =
            CANSparkMax(intakeFollowCanId, CANSparkLowLevel.MotorType.kBrushless)

    // private val limitSwitch = DigitalInput(limitSwitchId)
    private var state = State.Looking

    init {
        intakeSparkMax.restoreFactoryDefaults()
        intakeFollowSparkMax.restoreFactoryDefaults()
        intakeFollowSparkMax.follow(intakeSparkMax)
    }

    fun startIntaking() {
        intakeSparkMax.set(Constants.Intake.speed)        
    }

    fun stopIntaking() {
        intakeSparkMax.set(0.0);
    }

    fun intake(): Command {
        var parent = this;
        return Commands.startEnd(object: Runnable {
                override fun run() {
                    parent.startIntaking();
                    SmartDashboard.putBoolean("shooting", true)
                }
            },object: Runnable {
                override fun run() {
                    parent.stopIntaking()
                    SmartDashboard.putBoolean("shooting", false)
                }
            },parent);
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        // when (state) {
        // State.Idle -> {
        //     if (!limitSwitch.get()) {
        //         state = State.Looking
        //     }
        //     intakeSparkMax.set(0.0)
        // }

        // State.Looking -> {
        //     intakeSparkMax.set(Constants.Intake.speed)

        //     if (limitSwitch.get()) {
        //         state = State.Holding
        //     }
        // }
        // State.Holding -> {
        //     intakeSparkMax.set(0.0)

        //     if (!limitSwitch.get()) {
        //         state = State.Looking
        //     }
        // }
    }
}
