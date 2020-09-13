import inverse_kinematics.Joint
import inverse_kinematics.Skeleton
import org.openrndr.application
import org.openrndr.color.ColorRGBa
import org.openrndr.math.Vector2

fun main() = application {
    configure {
        width = 1366
        height = 768
    }

    program {

        val joint1 = Joint(Vector2(width / 2.0, height / 2.0))
        val joint2 = Joint(joint1.position + Vector2.UNIT_X * 50.0)
        val joint3 = Joint(joint2.position + Vector2.UNIT_X * 50.0)
        val joint4 = Joint(joint3.position + Vector2.UNIT_X * 50.0)
        val joint5 = Joint(joint4.position + Vector2.UNIT_X * 50.0)
        joint1.attachJoint(joint2)
        joint2.attachJoint(joint3)
        joint3.attachJoint(joint4)
        joint4.attachJoint(joint5)

        val skeleton = Skeleton(joint1)

        extend {
            drawer.clear(ColorRGBa.WHITE)

            skeleton.root.position = Vector2(width / 2.0, (frameCount % height).toDouble())
            skeleton.moveTowards(mouse.position)
            skeleton.draw(drawer)
        }
    }
}
