package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants
import frc.robot.utils.MovingAverage
import kotlin.sequences.sequence

class Shooter(
        private val shooterCanID: Int,
        private val secondaryShooterID: Int,
        private val shooterJointCanID: Int,
        private val shooterJoint2CanID: Int,
        private val intakeMotorID: Int
) : SubsystemBase() {

    private var speed = 0.0

    private var desiredAngle = 0.0

    private val shooterSparkMax = CANSparkFlex(shooterCanID, CANSparkLowLevel.MotorType.kBrushless)
    private val followerSparkMax =
            CANSparkFlex(secondaryShooterID, CANSparkLowLevel.MotorType.kBrushless)

    private val jointMotor1 = CANSparkMax(shooterJointCanID, CANSparkLowLevel.MotorType.kBrushless)
    private val jointMotor2 = CANSparkMax(shooterJoint2CanID, CANSparkLowLevel.MotorType.kBrushless)

    private val intakingMotor = CANSparkFlex(15, CANSparkLowLevel.MotorType.kBrushless)

    private var currentSetPoint = 0.0
    private var currentAverage = MovingAverage(10)

    private var topWheels = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, 0.00176)

    private var joint =
            SingleJointedArmSim(DCMotor.getNEO(2), 30.0, 0.616, 0.43, 0.0, Math.PI / 2.0, true, 0.0)

    private var sim_top_pid =
            PIDController(
                    Constants.Shooter.sim_pid.kP,
                    Constants.Shooter.sim_pid.kI,
                    Constants.Shooter.sim_pid.kD
            )

    private var sim_bottom_pid =
            PIDController(
                    Constants.Shooter.sim_pid.kP,
                    Constants.Shooter.sim_pid.kI,
                    Constants.Shooter.sim_pid.kD
            )

    private var sim_joint_pid =
            PIDController(
                    Constants.Shooter.sim_join_pid.kP,
                    Constants.Shooter.sim_join_pid.kI,
                    Constants.Shooter.sim_join_pid.kD
            )

    private var bottomWheels = FlywheelSim(DCMotor.getNEO(1), 1.0, 14.4)

    init {
        shooterSparkMax.restoreFactoryDefaults()
        followerSparkMax.restoreFactoryDefaults()
        jointMotor1.restoreFactoryDefaults()
        jointMotor2.restoreFactoryDefaults()
        intakingMotor.restoreFactoryDefaults()

        jointMotor1.pidController.p = Constants.Shooter.join_pid.kP
        jointMotor1.pidController.i = Constants.Shooter.join_pid.kI
        jointMotor1.pidController.d = Constants.Shooter.join_pid.kD

        jointMotor2.follow(jointMotor1)

        jointMotor1.encoder.positionConversionFactor = Constants.Shooter.positionConversionFactor

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
        SmartDashboard.putNumber("current set speed launcher", 0.0)
        SmartDashboard.putNumber("current set joint location launcher", 0.0)
        SmartDashboard.putData("joint pid", sim_joint_pid)
        followerSparkMax.follow(shooterSparkMax)
        //shooterSparkMax.set(Constants.Shooter.speed)
        //intakingMotor.set(0.75)
    }

    override fun periodic() {
        if (RobotBase.isReal()) {
            speed = shooterSparkMax.encoder.velocity
        }

        currentAverage.addValue(speed)
        SmartDashboard.putNumber("current motor speed launcher", speed)
        currentSetPoint = SmartDashboard.getNumber("current set speed launcher", 0.0)
        desiredAngle = SmartDashboard.getNumber("current set joint location launcher", 0.0)
    }

    fun setSpeed(speed: Double) {
        currentSetPoint = speed
        shooterSparkMax.pidController.setReference(speed, CANSparkBase.ControlType.kSmartVelocity)
    }

    fun setAngle(angle: Double) {
        jointMotor1.pidController.setReference(angle, CANSparkBase.ControlType.kPosition)
        desiredAngle = angle
    }

    fun atSpeed(): Boolean {
        val av = currentAverage.getAverage()
        return Math.abs(currentSetPoint - av) <= Constants.Shooter.tolerance
    }

    fun setIntakingSpeed(speed: Double) {
        intakingMotor.set(speed);
    }

    fun intake() {
        setIntakingSpeed(Constants.Shooter.intakeSpeed);
    }

    fun stopIntaking() {
        setIntakingSpeed(0.0)
    }

    fun intakeButtonCommand(): Command {
        val parent = this;
        return Commands.startEnd(object: Runnable {
                override fun run() {
                    parent.intake();
                }
            },object: Runnable {
                override fun run() {
                    parent.stopIntaking()
                }
            },this);
    }

    fun startShooting() {
        shooterSparkMax.set(Constants.Shooter.speed);
    }
    
    fun stopShooting() {
        shooterSparkMax.set(0.0);
    }

    fun shootButton(): Command {
        var parent = this;
        return Commands.startEnd(object: Runnable {
                override fun run() {
                    parent.startShooting();
                    SmartDashboard.putBoolean("shooting", true)
                }
            },object: Runnable {
                override fun run() {
                        parent.stopShooting()
                        SmartDashboard.putBoolean("shooting", false)
                }
            },parent);
    }

    override fun simulationPeriodic() {
        topWheels.setInputVoltage(
                sim_top_pid
                        .calculate(topWheels.angularVelocityRPM, currentSetPoint)
                        .coerceIn(-1.0..1.0) * RobotController.getInputVoltage()
        )

        bottomWheels.setInputVoltage(
                sim_bottom_pid
                        .calculate(topWheels.angularVelocityRPM, currentSetPoint)
                        .coerceIn(-1.0..1.0) * RobotController.getInputVoltage()
        )

        val voltage =
                sim_joint_pid.calculate(joint.angleRads, desiredAngle).coerceIn(-1.0, 1.0) *
                        RobotController.getInputVoltage()

        SmartDashboard.putNumber("angle of arm", joint.angleRads)

        SmartDashboard.putNumber("input to arm voltage", voltage)

        joint.setInputVoltage(voltage)

        topWheels.update(0.02)
        bottomWheels.update(0.02)
        joint.update(0.02)

        jointMotor1.encoder.position = joint.angleRads

        speed = topWheels.angularVelocityRPM
    }
}
