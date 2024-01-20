package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class Shooter(private val shooterCanID: Int) : SubsystemBase() {

    private var speed = 0
    private val shooterSparkMax = CANSparkFlex(shooterCanID, CANSparkLowLevel.MotorType.kBrushless)

    init {
        shooterSparkMax.restoreFactoryDefaults()
        shooterSparkMax.setSmartCurrentLimit(40, 40, 10_000_000)
        shooterSparkMax.pidController.setP(Constants.Shooter.p)
        shooterSparkMax.pidController.setI(Constants.Shooter.i)
        shooterSparkMax.pidController.setD(Constants.Shooter.d)
    }

    override fun periodic() {}

    fun setSpeed(speed: Double) {
        shooterSparkMax.pidController.setReference(speed, CANSparkBase.ControlType.kSmartVelocity)
    }
}
