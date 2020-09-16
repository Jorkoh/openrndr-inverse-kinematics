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

        val head = Joint(Vector2(0.0, 0.0), "head")
        val spine1 = Joint(head.position + Vector2.UNIT_X * 50.0, "spine1")
        val spine2 = Joint(head.position + Vector2.UNIT_X * 100.0, "spine2")
        val spine3 = Joint(head.position + Vector2.UNIT_X * 150.0, "spine3")
        val spine4 = Joint(head.position + Vector2.UNIT_X * 200.0, "spine4")
        val spine5 = Joint(head.position + Vector2.UNIT_X * 250.0, "spine5")
        val spine6 = Joint(head.position + Vector2.UNIT_X * 300.0, "spine6")
        val spine7 = Joint(head.position + Vector2.UNIT_X * 350.0, "spine7")
        head.attachJoint(spine1)
        spine1.attachJoint(spine2, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine2.attachJoint(spine3, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine3.attachJoint(spine4, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine4.attachJoint(spine5, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine5.attachJoint(spine6, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine6.attachJoint(spine7, Pair(150.0.toRadians(), 210.0.toRadians()))

        val skeleton = Skeleton(head)

        var tracking = true
        mouse.clicked.listen {
            tracking = !tracking
        }

        extend {
            // Move skeleton root
            skeleton.root.position = mouse.position
            // Solve IK
            skeleton.solve(
                if (tracking) {
                    mapOf(Pair(spine7, Vector2(width / 2.0, height / 2.0)))
                } else {
                    mapOf()
                }
            )

            //Draw
            drawer.clear(ColorRGBa.WHITE)
            drawer.fontMap = font
            drawer.stroke = ColorRGBa.BLACK
            drawer.fill = ColorRGBa.BLACK
            skeleton.draw(drawer)

            drawer.stroke = ColorRGBa.RED
            drawer.fill = ColorRGBa.RED
            drawer.circle(Vector2(width / 2.0, height / 2.0), 4.0)
            drawer.text("target", Vector2(width / 2.0, height / 2.0) + Vector2.ONE * 5.0)
        }
    }
}
