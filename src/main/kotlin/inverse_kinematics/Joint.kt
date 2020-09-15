package inverse_kinematics

import org.openrndr.math.Vector2

data class Joint(var position: Vector2, val id: String) {
    var parentJoint: Joint? = null
    var constraintsWithParent: Pair<Double, Double>? = null
    var attachedJoints: MutableList<Joint> = mutableListOf()

    fun attachJoint(joint: Joint, constraints: Pair<Double, Double>? = null) {
        attachedJoints.add(joint)
        joint.parentJoint = this

        if (constraints != null) {
            require(parentJoint != null) { "Can't form a constraint with the root joint" }
            joint.constraintsWithParent = constraints
        }
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as Joint

        if (id != other.id) return false

        return true
    }

    override fun hashCode(): Int {
        return id.hashCode()
    }


}
