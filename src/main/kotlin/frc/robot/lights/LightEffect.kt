package frc.robot.lights

import edu.wpi.first.wpilibj.AddressableLEDBuffer

interface LightEffect {
    fun nextBuffer(tick: Int, addressableLEDBuffer: AddressableLEDBuffer)
}
