import inverse_kinematics.Joint
import inverse_kinematics.Skeleton
import org.openrndr.draw.Drawer
import org.openrndr.draw.loadFont
import org.openrndr.math.Vector2
import org.openrndr.shape.contour

val font = loadFont("data/default.otf", 12.0)

fun Skeleton.draw(drawer: Drawer) {
    root.draw(drawer)
}

// TODO draw constraints
private fun Joint.draw(drawer: Drawer) {
    drawer.circle(position, 4.0)
    drawer.text(id, position + Vector2.ONE * 5.0)
    parentJoint?.let { parentJoint ->
        // Don't draw line if it has no parent
        drawer.lineSegment(parentJoint.position, position)
    }
    attachedJoints.forEach { it.draw(drawer) }
}