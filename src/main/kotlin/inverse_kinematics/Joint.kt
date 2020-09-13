package inverse_kinematics

import org.openrndr.extra.gui.addTo
import org.openrndr.math.Vector2

data class Joint(
    var position: Vector2,
    var parentJoint: Joint? = null
) {
    var attachedJoints: MutableList<Joint> = mutableListOf()
    var attachedJointsConstraints: MutableList<Pair<Double, Double>> = mutableListOf()

    fun attachJoint(joint: Joint, constraints : Pair<Double, Double>) {
        attachedJoints.add(joint)
        attachedJointsConstraints.add(constraints)
        joint.parentJoint = this
    }
}
