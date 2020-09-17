import gecko.World
import org.openrndr.application
import org.openrndr.color.ColorRGBa
import org.openrndr.math.Vector2

fun main() = application {
    configure {
        width = World.WIDTH.toInt()
        height = World.HEIGHT.toInt()
    }

    program {

        extend {
            World.update(mouse.position)

            //Draw
            drawer.clear(ColorRGBa.WHITE)
            drawer.fontMap = font
            drawer.stroke = ColorRGBa.BLACK
            drawer.fill = ColorRGBa.BLACK
            World.gecko.skeleton.draw(drawer)
        }
    }
}
