package inverse_kinematics

import org.openrndr.math.Vector2

data class Skeleton(
    val root: Joint
) {
    private val links: Map<Pair<Joint, Joint>, Double>

    init {
        val tempLinks = mutableMapOf<Pair<Joint, Joint>, Double>()
        val tempEnds = mutableListOf<Joint>()
        addLinksAndEnds(root, tempLinks, tempEnds)
        links = tempLinks
    }

    // Note: it tries to solve even on cases where targets are unreachable
    fun solve(targets: Map<Joint, Vector2>) {
        val rootPosition = root.position

        val tolerance = 1.0
        val maxIterations = 10
        var iteration = 0

        // Use do-while to adjust even when the end effector is on target since the root can move
        do {
            // STAGE 1: FORWARD REACHING
            val subBasePositions = mutableMapOf<Joint, MutableList<Vector2>>()

            val forwardStuffToCalculate = targets.keys.toMutableList()
            while (forwardStuffToCalculate.size > 0) {
                val previous = forwardStuffToCalculate.first()
                forwardStuffToCalculate.remove(previous)
                val current = previous.parentJoint ?: continue
                if (previous.attachedJoints.size == 0) {
                    // It's an end
                    previous.position = requireNotNull(targets[previous])
                }

                val length = requireNotNull(links[Pair(current, previous)]) { "Missing a link" }
                val position = previous.position + (current.position - previous.position).setLength(length)
                if (current.attachedJoints.size > 1) {
                    // it's a sub-base
                    val positions = subBasePositions.getOrDefault(current, mutableListOf())
                    positions.add(position)

                    if (positions.size == current.attachedJoints.size) {
                        // All of the joints connecting to this sub-base have been calculated, position is centroid
                        var sum = Vector2.ZERO
                        positions.forEach { sum += it }
                        current.position = sum / positions.size.toDouble()

                        forwardStuffToCalculate.add(current)
                    } else {
                        // There are still joints connecting to this sub-base that need to be calculated
                        subBasePositions[current] = positions
                    }
                } else {
                    // It's not a sub-base, just calculate it
                    current.position = position

                    forwardStuffToCalculate.add(current)
                }
            }

            // STAGE 2: BACKWARD REACHING
            root.position = rootPosition
            val backwardStuffToCalculate = root.attachedJoints.toMutableList()
            while (backwardStuffToCalculate.size > 0) {
                val current = backwardStuffToCalculate.first()
                backwardStuffToCalculate.remove(current)
                val previous = requireNotNull(current.parentJoint) { "Malformed skeleton" }

                val constraints = current.constraintsWithParent
                val previousParent = previous.parentJoint
                var constrainedDirection = current.position - previous.position
                if (constraints != null && previousParent != null) {
                    // This joint is constrained, apply constraints
                    constrainedDirection = constrainedDirection.clampAngleDifference(
                        previousParent.position - previous.position,
                        constraints.first,
                        constraints.second
                    )
                }

                val length = links[Pair(previous, current)] ?: continue
                current.position = previous.position + constrainedDirection.setLength(length)

                backwardStuffToCalculate.addAll(current.attachedJoints)
            }
            iteration++
        } while (!targets.allOnTarget(tolerance) && iteration < maxIterations)

        targets.roundToTarget(tolerance)
    }

    private fun Map<Joint, Vector2>.allOnTarget(tolerance: Double): Boolean {
        return all { entry ->entry.key.position.distanceTo(entry.value) <= tolerance        }
    }

    private fun Map<Joint, Vector2>.roundToTarget(tolerance: Double) {
        forEach { entry ->
            if (entry.key.position.distanceTo(entry.value) <= tolerance) {
                entry.key.position = entry.value
            }
        }
    }

    private fun addLinksAndEnds(
        joint: Joint,
        tempLinks: MutableMap<Pair<Joint, Joint>, Double>,
        tempEnds: MutableList<Joint>
    ) {
        joint.parentJoint?.let { parent ->
            tempLinks[Pair(parent, joint)] = parent.position.distanceTo(joint.position)
        }
        if (joint.attachedJoints.isEmpty()) {
            tempEnds.add(joint)
        } else {
            joint.attachedJoints.forEach { addLinksAndEnds(it, tempLinks, tempEnds) }
        }
    }
}