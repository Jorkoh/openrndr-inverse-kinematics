package inverse_kinematics

import org.openrndr.math.Vector2

data class Skeleton(
    val root: Joint
) {
    private val linkLengths: List<Double>

    init {
        val joints = mutableListOf(root)
        addJoints(root, joints)
        linkLengths = joints.drop(1).mapIndexed { index, joint ->
            joints[index].position.distanceTo(joint.position)
        }
    }

    fun moveTowards(target: Vector2) {
        val rootPosition = root.position
        // TODO support sub-bases
        val joints = mutableListOf(root)
        addJoints(root, joints)
        val constraints = mutableListOf<Pair<Double, Double>>()
        addConstraints(root, constraints)

        val tolerance = 1.0
        val maxIterations = 10
        var iteration = 0

        // Use do-while to adjust even when the end effector is on target since the root can move
        do {
            // STAGE 1: FORWARD REACHING
            for (i in joints.indices.reversed()) {
                if (i == joints.size - 1) {
                    joints[i].position = target
                } else {
                    // Don't need to adjust the first one on the forward pass
                    if (i == 0) continue

                    val current = joints[i].position
                    val previous = joints[i + 1].position
                    joints[i].position = previous + (current - previous).setLength(linkLengths[i])
                }
            }
            // STAGE 2: BACKWARD REACHING
            for (i in joints.indices) {
                if (i == 0) {
                    joints[i].position = rootPosition
                } else {
                    val current = if (i < joints.size - 1) joints[i].position else target
                    val previous = joints[i - 1].position
                    // Clamp according to joint constraints
                    val constrainedDirection = if (i == 1) {
                        // No previous link on the skeleton, no need to clamp
                        current - previous
                    } else {
                        (current - previous).clampAngleDifference(
                             previous - joints[i - 2].position,
                            constraints[i-1].first,
                            constraints[i-1].second
                        )
                    }

                    joints[i].position = previous + constrainedDirection.setLength(linkLengths[i - 1])
                }
            }
            iteration++
        } while (joints.last().position.distanceTo(target) > tolerance && iteration < maxIterations)
    }

    private fun addJoints(joint: Joint, joints: MutableList<Joint>) {
        joints.addAll(joint.attachedJoints)
        joint.attachedJoints.forEach { addJoints(it, joints) }
    }

    private fun addConstraints(joint: Joint, constraints: MutableList<Pair<Double, Double>>) {
        constraints.addAll(joint.attachedJointsConstraints)
        joint.attachedJoints.forEach { addConstraints(it, constraints) }
    }
}