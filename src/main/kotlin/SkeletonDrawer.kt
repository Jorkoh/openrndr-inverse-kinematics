import inverse_kinematics.Joint
import inverse_kinematics.Skeleton
import org.openrndr.draw.Drawer


fun Skeleton.draw(drawer: Drawer) {
    root.attachedJoints.forEach { it.draw(drawer) }
}

private fun Joint.draw(drawer: Drawer) {
    // Don't draw if it has no parent
    parentJoint?.let { parentJoint ->
        drawer.circle(position, 5.0)
        drawer.lineSegment(parentJoint.position, position)
        attachedJoints.forEach { it.draw(drawer) }
    }
}