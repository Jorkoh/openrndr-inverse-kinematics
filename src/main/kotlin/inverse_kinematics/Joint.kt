package inverse_kinematics

import org.openrndr.math.Vector2

data class Joint(
    var position: Vector2,
    var parentJoint: Joint? = null
) {
    var attachedJoints: MutableList<Joint> = mutableListOf()

    fun attachJoint(joint: Joint) {
        attachedJoints.add(joint)
        joint.parentJoint = this
    }
}
