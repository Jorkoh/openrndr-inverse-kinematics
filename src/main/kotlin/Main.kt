import inverse_kinematics.Joint
import inverse_kinematics.Skeleton
import inverse_kinematics.toRadians
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
        val joint6 = Joint(joint5.position + Vector2.UNIT_X * 50.0)
        joint1.attachJoint(joint2, Pair(120.0.toRadians(), 240.0.toRadians()))
        joint2.attachJoint(joint3, Pair(120.0.toRadians(), 240.0.toRadians()))
        joint3.attachJoint(joint4, Pair(120.0.toRadians(), 240.0.toRadians()))
        joint4.attachJoint(joint5, Pair(120.0.toRadians(), 240.0.toRadians()))
        joint5.attachJoint(joint6, Pair(120.0.toRadians(), 240.0.toRadians()))

        val skeleton = Skeleton(joint1)

        extend {
            drawer.clear(ColorRGBa.WHITE)

            skeleton.root.position = Vector2(width / 2.0, (frameCount % height).toDouble())
            skeleton.moveTowards(mouse.position)
            skeleton.draw(drawer)
        }
    }
}
