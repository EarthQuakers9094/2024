package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.Constants
import frc.robot.utils.MovingAverage
import java.util.function.BooleanSupplier
import com.revrobotics.SparkPIDController

class Shooter(
        private val shooterCanID: Int,
        private val secondaryShooterID: Int,
        private val shooterJointCanID: Int,
        private val intakeMotorID: Int,
) : SubsystemBase() {

    private var speed = 0.0

    private var desiredAngle = 0.0

    private var angleRollingAverage = MovingAverage(20)

    private val shooterSparkMax = CANSparkFlex(shooterCanID, CANSparkLowLevel.MotorType.kBrushless)
    private val followerSparkMax =
            CANSparkFlex(secondaryShooterID, CANSparkLowLevel.MotorType.kBrushless)

    private val jointMotor1 = CANSparkMax(shooterJointCanID, CANSparkLowLevel.MotorType.kBrushless)

    private val intakingMotor = CANSparkFlex(intakeMotorID, CANSparkLowLevel.MotorType.kBrushless)

    private var currentSetPoint = 0.0
    private var currentAverage = MovingAverage(10)

    private var topWheels = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, 0.00176)

    private var joint =
            SingleJointedArmSim(DCMotor.getNEO(2), 30.0, 0.2, 0.43, 0.0, Math.PI / 2.0, true, 0.0)

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

    private var inSensor = DigitalInput(Constants.Shooter.inSensorID)

    private var currentSetSpeed = 0.0;

    init {
        shooterSparkMax.restoreFactoryDefaults()
        followerSparkMax.restoreFactoryDefaults()
        jointMotor1.restoreFactoryDefaults()
        intakingMotor.restoreFactoryDefaults()

        while (!jointMotor1.inverted) {
            jointMotor1.inverted = true
        }

        jointMotor1.pidController.p = Constants.Shooter.join_pid.kP
        jointMotor1.pidController.i = Constants.Shooter.join_pid.kI
        jointMotor1.pidController.d = Constants.Shooter.join_pid.kD

        jointMotor1.encoder.positionConversionFactor = Constants.Shooter.positionConversionFactor

        jointMotor1.encoder.position = Constants.Shooter.startAngle;

        jointMotor1.pidController.setSmartMotionMaxVelocity(1.5,0);
        jointMotor1.pidController.setSmartMotionMaxAccel(0.0001,0);
        jointMotor1.pidController.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0);

        followerSparkMax.follow(shooterSparkMax, true)

        shooterSparkMax.encoder.velocityConversionFactor =
                Constants.Shooter.velocityConversionFactor

        shooterSparkMax.setSmartCurrentLimit(40, 40, 10_000_000)
        followerSparkMax.setSmartCurrentLimit(40, 40, 10_000_000)
        intakingMotor.setSmartCurrentLimit(40, 40, 10_000_000)

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
        // shooterSparkMax.set(Constants.Shooter.speed)
        // intakingMotor.set(0.75)
    }

    override fun periodic() {
        if (RobotBase.isReal()) {
            speed = shooterSparkMax.encoder.velocity
        }

        angleRollingAverage.addValue(jointMotor1.encoder.position)

        SmartDashboard.putNumber("shooter angle", jointMotor1.encoder.position)

        SmartDashboard.putNumber("current motor speed launcher", speed)
        SmartDashboard.putNumber("shooter speed", shooterSparkMax.encoder.velocity)

        SmartDashboard.putBoolean("beam break", noteIn());
    }

    fun setSpeed(speed: Double) {
        currentSetPoint = speed
        shooterSparkMax.pidController.setReference(speed, CANSparkBase.ControlType.kSmartVelocity)
    }

    fun setAngle(angle: Double) {
        jointMotor1.pidController.setReference(angle, CANSparkBase.ControlType.kPosition)
        desiredAngle = angle
        SmartDashboard.putNumber("shooter desired angle", desiredAngle)
        DriverStation.reportError("hello :3 from shooter", true)
    }

    fun atAngle():Boolean {
        return Math.abs(angleRollingAverage.getAverage() - desiredAngle) <= 0.05;
    }

    fun setIntakingSpeed(speed: Double) {
        intakingMotor.set(speed)
    }

    fun intake() {
        setIntakingSpeed(Constants.Shooter.intakeSpeed)
    }

    fun stopIntaking() {
        setIntakingSpeed(0.0)
    }

    fun intakeButtonCommand(): Command {
        val parent = this
        return Commands.startEnd(
                object : Runnable {
                    override fun run() {
                        parent.intake()
                    }
                },
                object : Runnable {
                    override fun run() {
                        parent.stopIntaking()
                    }
                },
                this
        )
    }

    fun startShooting(amp: Boolean) {
        val speed =
                if (amp) {
                    Constants.Shooter.ampSpeed
                } else {
                    Constants.Shooter.speed
                }
        currentSetSpeed = speed

        shooterSparkMax.set(speed)
    }

    fun stopShooting() {
        shooterSparkMax.set(0.0)
        currentSetSpeed = 0.0
    }

    fun shootButton(): Command {
        var parent = this
        return Commands.startEnd(
                object : Runnable {
                    override fun run() {
                        parent.startShooting(false)
                        parent.intake()
                    }
                },
                object : Runnable {
                    override fun run() {
                        parent.stopShooting()
                        parent.stopIntaking()
                    }
                },
                parent
        )
    }

    fun back(): Unit {
        shooterSparkMax.set(0.5)
        currentSetSpeed = 0.1
        intakingMotor.set(-0.1)
    }

    fun backButton(): Command {
        var parent = this
        return Commands.startEnd(
                object : Runnable {
                    override fun run() {
                        back();
                    }
                },
                object : Runnable {
                    override fun run() {
                        parent.stopShooting()
                        parent.stopIntaking()
                    }
                },
                parent
        )
    }

    fun shootTime(intake: Intake, amp: Boolean): Command {
        val parent = this

        val supplier = { this.atSpeed(amp) }

        val command =
                Commands.sequence(
                        InstantCommand(
                                object : Runnable {
                                    override fun run() {
                                        parent.startShooting(amp)
                                    }
                                }
                        ),
                        WaitUntilCommand(supplier),
                        InstantCommand(
                                object : Runnable {
                                    override fun run() {
                                        parent.intake()
                                        intake.startIntaking()
                                    }
                                }
                        ),
                        WaitCommand(Constants.Shooter.shootTime),
                        InstantCommand(
                                object : Runnable {
                                    override fun run() {
                                        parent.stopIntaking()
                                        parent.stopShooting()
                                        intake.stopIntaking()
                                    }
                                }
                        )
                )

        command.addRequirements(this)

        return command
    }

    fun pickup(): Command {
        val parent = this

        val supplier: BooleanSupplier = BooleanSupplier { this.noteIn() }

        val command =
                Commands.sequence(
                        InstantCommand(
                                object : Runnable {
                                    override fun run() {
                                        parent.intake()
                                    }
                                }
                        ),
                        WaitUntilCommand(supplier)
                )

        command.addRequirements(this)

        return command.andThen(
                InstantCommand(
                        object : Runnable {
                            override fun run() {
                                parent.stopIntaking()
                            }
                        },
                        this
                )
        )
    }

    override fun simulationPeriodic() {
        SmartDashboard.putNumber("currentSetSpeed", currentSetSpeed)
        topWheels.setInputVoltage(currentSetSpeed * RobotController.getInputVoltage())

        bottomWheels.setInputVoltage(-currentSetSpeed * RobotController.getInputVoltage())

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

    fun noteIn(): Boolean {
        return !inSensor.get()
    }

    fun atSpeed(amp: Boolean): Boolean {
        if (RobotBase.isReal()) {
            speed = shooterSparkMax.encoder.velocity
        }

        return if (amp) {
            speed <= Constants.Shooter.ampShootingRotationSpeed
        } else {
            speed <= -4500.0
        }
    }
}
