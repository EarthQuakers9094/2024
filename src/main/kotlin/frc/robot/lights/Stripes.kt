package frc.robot.lights

import frc.robot.lights.Color

import edu.wpi.first.wpilibj.AddressableLEDBuffer

class Stripes(private val colors: List<Color>) : LightEffect {
    override fun nextBuffer(tick: Int, addressableLEDBuffer: AddressableLEDBuffer) {
        val stripes = colors.size;
        for (i in 0 until addressableLEDBuffer.length) {
            val index:Int = i/stripes;
            val color = colors[index];
            addressableLEDBuffer.setRGB(i, color.r, color.g, color.b)
        }
    }

    companion object {
        fun lesbian() : Stripes {
            return Stripes([Color(0xd6,0x28,0), 
                            Color(0xff,0x9b,0x56),
                            Color(0xff,0xff,0xff),
                            Color(0xd4,0x62,0xa6),
                            Color(0xa4,0x00,0x62)]);
        }

        fun india() : Stripes {
            return Stripes([
                    Color(0xff,0x9a,0x2f),
                    Color(0xff,0xff,0xff),
                    Color(0xa,0x89,1)
                ]);
        }

        fun gay() : Stripes {
            return Stripes([
                Color(0xe4,0x03,0x03),
                Color(0xff,0x8c,0x00),
                Color(0xff,0xed,0x00),
                Color(0x00,0x80,0x26),
                Color(0x00,0x4d,0xff),
                Color(0x75,0x07,0x87)
            ])
        }

        fun trans() : Stripes {
            return Stripes([
                Color(0x55,0xcd,0xfd),
                Color(0xf6,0xaa,0xb7),
                Color(0xff,0xff,0xff),
                Color(0xf6,0xaa,0xb7),
                Color(0x55,0xcd,0xfd)
            ])
        }

        fun bisexual() : Stripes {
            return Stripes(
                [
                    Color(0xd7,0x00,0x71),
                    Color(0x9c,0x4e,0x97),
                    Color(0x00,0x35,0xaa)
                ]
            )
        }
    }
}
