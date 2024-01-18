package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lights.LightEffect
import frc.robot.lights.NoLightEffect

class Lights(private val stripPWM: Int, private val stripLength: Int) : SubsystemBase() {

    private val ledStrip =
            AddressableLED(stripPWM).let {
                it.setLength(stripLength)
                it
            }
    private var ledbuffer = AddressableLEDBuffer(stripLength)
    var currentEffect: LightEffect = NoLightEffect()
        set(value) {
            field = value
            tick = 0
        }
    private var tick = 0

    init {
        ledStrip.setLength(stripLength)
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        tick++
        currentEffect.nextBuffer(tick, ledbuffer)
        ledStrip.setData(ledbuffer)
    }
}
