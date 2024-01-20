package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.utils.MovingAverage

class Shooter(private val shooterCanID: Int, private val secondaryShooterID: Int) :
        SubsystemBase() {

    private var speed = 0
    private val shooterSparkMax = CANSparkFlex(shooterCanID, CANSparkLowLevel.MotorType.kBrushless)
    private val followerSparkMax =
            CANSparkFlex(secondaryShooterID, CANSparkLowLevel.MotorType.kBrushless)
    private var currentSetPoint = 0.0
    private var currentAverage = MovingAverage(10)

    init {
        shooterSparkMax.restoreFactoryDefaults()
        followerSparkMax.restoreFactoryDefaults()

        followerSparkMax.follow(shooterSparkMax, true)

        shooterSparkMax.encoder.velocityConversionFactor =
                Constants.Shooter.velocityConversionFactor

        shooterSparkMax.setSmartCurrentLimit(40, 40, 10_000_000)
        followerSparkMax.setSmartCurrentLimit(40, 40, 10_000_000)

        shooterSparkMax.pidController.setP(Constants.Shooter.p)
        shooterSparkMax.pidController.setI(Constants.Shooter.i)
        shooterSparkMax.pidController.setD(Constants.Shooter.d)

        followerSparkMax.pidController.setP(Constants.Shooter.p)
        followerSparkMax.pidController.setI(Constants.Shooter.i)
        followerSparkMax.pidController.setD(Constants.Shooter.d)
    }

    override fun periodic() {
        currentAverage.addValue(shooterSparkMax.encoder.velocity)
    }

    fun setSpeed(speed: Double) {
        currentSetPoint = speed
        shooterSparkMax.pidController.setReference(speed, CANSparkBase.ControlType.kSmartVelocity)
    }

    fun atSpeed(): Boolean {
        val av = currentAverage.getAverage()
        return Math.abs(currentSetPoint - av) <= Constants.Shooter.tolerance
    }
}
