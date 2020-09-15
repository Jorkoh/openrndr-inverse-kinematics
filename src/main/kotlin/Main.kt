import inverse_kinematics.Joint
import inverse_kinematics.Skeleton
import inverse_kinematics.toRadians
import org.openrndr.application
import org.openrndr.color.ColorRGBa
import org.openrndr.math.Vector2
import kotlin.math.cos
import kotlin.math.sin

fun main() = application {
    configure {
        width = 1366
        height = 768
    }

    program {

        val root = Joint(Vector2(width / 2.0, height / 2.0), "root")
        val middle1 = Joint(root.position + Vector2.UNIT_X * 100.0, "middle1")
        val middle2A = Joint(root.position + Vector2.UNIT_X * 150.0, "middle2A")
        val middle2B = Joint(root.position + Vector2.UNIT_X * 175.0, "middle2B")
        val middle3A = Joint(root.position + Vector2.UNIT_X * 200.0, "middle3A")
        val end1 = Joint(root.position + Vector2.UNIT_X * 220.0, "end1")
        val end2 = Joint(root.position + Vector2.UNIT_X * 240.0, "end2")
        val end3 = Joint(root.position + Vector2.UNIT_X * 200.0, "end3")
        root.attachJoint(middle1)
        middle1.attachJoint(middle2A, Pair(0.0.toRadians(), 360.0.toRadians()))
        middle2A.attachJoint(middle3A, Pair(0.0.toRadians(), 360.0.toRadians()))
        middle1.attachJoint(middle2B, Pair(0.0.toRadians(), 360.0.toRadians()))
        middle3A.attachJoint(end1, Pair(0.0.toRadians(), 360.0.toRadians()))
        middle3A.attachJoint(end2, Pair(0.0.toRadians(), 360.0.toRadians()))
        middle2B.attachJoint(end3, Pair(0.0.toRadians(), 360.0.toRadians()))

        val skeleton = Skeleton(root)

        val ends = listOf(end1, end2, end3)
        val targets = ends.map { Pair(it, it.position) }.toMap().toMutableMap()
        var mouseTargetIndex = 0
        mouse.clicked.listen {
            val end = ends[mouseTargetIndex++]
            targets[end] = mouse.position
            mouseTargetIndex %= ends.size
        }

        extend {
            // Move skeleton root
            skeleton.root.position = Vector2(
                width / 2.0 + 150 * sin(seconds * 0.8),
                height / 2.0 + 150 * cos(seconds * 0.8)
            )
            // Solve IK
            skeleton.solve(targets)

            //Draw
            drawer.clear(ColorRGBa.WHITE)
            drawer.fontMap = font
            drawer.stroke = ColorRGBa.BLACK
            drawer.fill = ColorRGBa.BLACK
            skeleton.draw(drawer)

            drawer.stroke = ColorRGBa.RED
            drawer.fill = ColorRGBa.RED
            for (target in targets) {
                drawer.circle(target.value, 4.0)
                drawer.text("${target.key.id} target", target.value + Vector2.ONE * 5.0)
            }
        }
    }
}
