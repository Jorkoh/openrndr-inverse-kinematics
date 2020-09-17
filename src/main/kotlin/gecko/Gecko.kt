package gecko

import inverse_kinematics.*
import org.openrndr.math.Vector2

class Gecko(spawnPosition: Vector2, var velocity: Vector2) {

    companion object {
        const val FRONT_FOOT_TARGET_ANGLE = 35.0
        const val FRONT_FOOT_TARGET_SEPARATION = 70.0

        const val BACK_FOOT_TARGET_ANGLE = 100.0
        const val BACK_FOOT_TARGET_SEPARATION = 50.0

        // Maximum head turn per time step
        const val MAX_TURN = 5.0

        const val MINIMUM_SPEED = 0.0
        const val MAXIMUM_SPEED = 4.0

        const val DRAG_FACTOR = 0.2

        const val OBJECTIVE_CHASING_FACTOR = 0.01
        const val OBJECTIVE_REACHED_RADIUS = 5.0

        fun spawnGecko(): Gecko {
            return Gecko(Vector2(World.WIDTH / 2, World.HEIGHT / 2), -Vector2.UNIT_X)
        }
    }

    val skeleton: Skeleton
    val forces: MutableList<Vector2> = mutableListOf()
    var feetTargets: Map<Joint, Vector2> = mapOf()

    private val frontHip: Joint
    private val frontLeftFoot: Joint
    private val frontRightFoot: Joint
    private val backHip: Joint
    private val backLeftFoot: Joint
    private val backRightFoot: Joint

    private var frontFootDown: Joint
    private var backFootDown: Joint

    init {
        val head = Joint(spawnPosition, "head")

        frontHip = Joint(head.position + Vector2.UNIT_X * 50.0, "spine1")
        val spine2 = Joint(frontHip.position + Vector2.UNIT_X * 50.0, "spine2")
        val spine3 = Joint(spine2.position + Vector2.UNIT_X * 50.0, "spine3")
        val spine4 = Joint(spine3.position + Vector2.UNIT_X * 50.0, "spine4")
        backHip = Joint(spine4.position + Vector2.UNIT_X * 50.0, "spine5")
        val spine6 = Joint(backHip.position + Vector2.UNIT_X * 50.0, "spine6")
        val spine7 = Joint(spine6.position + Vector2.UNIT_X * 50.0, "spine7")
        val spine8 = Joint(spine7.position + Vector2.UNIT_X * 50.0, "spine8")
        val spine9 = Joint(spine8.position + Vector2.UNIT_X * 50.0, "spine9")
        val spine10 = Joint(spine9.position + Vector2.UNIT_X * 50.0, "spine10")
        val spine11 = Joint(spine10.position + Vector2.UNIT_X * 50.0, "spine11")

        val frontLeftKnee = Joint(frontHip.position + Vector2.UNIT_Y * 30.0, "frontLeftKnee")
        frontLeftFoot = Joint(frontLeftKnee.position + Vector2.UNIT_Y * 40.0, "frontLeftFoot")
        val frontRightKnee = Joint(frontHip.position - Vector2.UNIT_Y * 30.0, "frontRightKnee")
        frontRightFoot = Joint(frontRightKnee.position - Vector2.UNIT_Y * 40.0, "frontRightFoot")

        val backLeftKnee = Joint(backHip.position + Vector2.UNIT_Y * 40.0, "backLeftKnee")
        backLeftFoot = Joint(backLeftKnee.position + Vector2.UNIT_Y * 60.0, "backLeftFoot")
        val backRightKnee = Joint(backHip.position - Vector2.UNIT_Y * 40.0, "backRightKnee")
        backRightFoot = Joint(backRightKnee.position - Vector2.UNIT_Y * 60.0, "backRightFoot")

        head.attachJoint(frontHip)

        frontHip.attachJoint(spine2, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine2.attachJoint(spine3, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine3.attachJoint(spine4, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine4.attachJoint(backHip, Pair(150.0.toRadians(), 210.0.toRadians()))
        backHip.attachJoint(spine6, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine6.attachJoint(spine7, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine7.attachJoint(spine8, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine8.attachJoint(spine9, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine9.attachJoint(spine10, Pair(150.0.toRadians(), 210.0.toRadians()))
        spine10.attachJoint(spine11, Pair(150.0.toRadians(), 210.0.toRadians()))

        frontHip.attachJoint(frontLeftKnee, Pair(190.0.toRadians(), 345.0.toRadians()))
        frontLeftKnee.attachJoint(frontLeftFoot, Pair(190.0.toRadians(), 345.0.toRadians()))
        frontHip.attachJoint(frontRightKnee, Pair(15.0.toRadians(), 170.0.toRadians()))
        frontRightKnee.attachJoint(frontRightFoot, Pair(15.0.toRadians(), 170.0.toRadians()))

        backHip.attachJoint(backLeftKnee, Pair(190.0.toRadians(), 345.0.toRadians()))
        backLeftKnee.attachJoint(backLeftFoot, Pair(10.0.toRadians(), 165.0.toRadians()))
        backHip.attachJoint(backRightKnee, Pair(15.0.toRadians(), 170.0.toRadians()))
        backRightKnee.attachJoint(backRightFoot, Pair(195.0.toRadians(), 350.0.toRadians()))

        skeleton = Skeleton(head)

        frontFootDown = frontLeftFoot
        backFootDown = backRightFoot
    }

    fun update(objective: Vector2) {
        calculateVelocity(objective)
        move()
    }

    private fun calculateVelocity(objective: Vector2) {
        // Calculate forces
        forces.clear()
        forces.add(dragForce())
        forces.add(objectiveChasingForce(objective))

        // Calculate updated velocity
        var newVelocity = velocity.copy()
        for (force in forces) {
            newVelocity += force
        }
        velocity = newVelocity
            .clampAbsoluteAngleDifference(velocity, MAX_TURN.toRadians())
            .clampLength(MINIMUM_SPEED, MAXIMUM_SPEED)
    }

    private fun dragForce(): Vector2 {
        return -velocity * DRAG_FACTOR
    }

    private fun objectiveChasingForce(objective: Vector2): Vector2 {
        return if ((objective - skeleton.root.position).length < OBJECTIVE_REACHED_RADIUS) {
            Vector2.ZERO
        } else {
            (objective - skeleton.root.position) * OBJECTIVE_CHASING_FACTOR
        }
    }

    private fun move() {
        skeleton.root.position += velocity
        feetTargets = calculateFeetTargets(velocity)
        skeleton.solve(feetTargets)

        if (frontFootDown == frontLeftFoot && feetTargets[frontLeftFoot] != frontLeftFoot.position) {
            frontFootDown = frontRightFoot
        } else if (frontFootDown == frontRightFoot && feetTargets[frontRightFoot] != frontRightFoot.position) {
            frontFootDown = frontLeftFoot
        }

        if (backFootDown == backLeftFoot && feetTargets[backLeftFoot] != backLeftFoot.position) {
            backFootDown = backRightFoot
        } else if (backFootDown == backRightFoot && feetTargets[backRightFoot] != backRightFoot.position) {
            backFootDown = backLeftFoot
        }
    }

    private fun calculateFeetTargets(velocity: Vector2): Map<Joint, Vector2> {
        val frontLeftFootTarget = if (frontFootDown == frontLeftFoot) {
            frontLeftFoot.position
        } else {
            val stepTarget = frontHip.position + Vector2.unitWithRadiansAngle(
                (frontHip.parentJoint!!.position - frontHip.position).angle() - FRONT_FOOT_TARGET_ANGLE.toRadians()
            ) * FRONT_FOOT_TARGET_SEPARATION

            if (frontLeftFoot.position.distanceTo(stepTarget) < velocity.length * 3.0) {
                stepTarget
            } else {
                frontLeftFoot.position + (stepTarget - frontLeftFoot.position).setLength(velocity.length * 3.0)
            }
        }

        val frontRightFootTarget = if (frontFootDown == frontRightFoot) {
            frontRightFoot.position
        } else {
            val stepTarget = frontHip.position + Vector2.unitWithRadiansAngle(
                (frontHip.parentJoint!!.position - frontHip.position).angle() + FRONT_FOOT_TARGET_ANGLE.toRadians()
            ) * FRONT_FOOT_TARGET_SEPARATION

            if (frontRightFoot.position.distanceTo(stepTarget) < velocity.length * 3.0) {
                stepTarget
            } else {
                frontRightFoot.position + (stepTarget - frontRightFoot.position).setLength(velocity.length * 3.0)
            }
        }

        val backLeftFootTarget = if (backFootDown == backLeftFoot) {
            backLeftFoot.position
        } else {
            val stepTarget = backHip.position + Vector2.unitWithRadiansAngle(
                (backHip.parentJoint!!.position - backHip.position).angle() - BACK_FOOT_TARGET_ANGLE.toRadians()
            ) * BACK_FOOT_TARGET_SEPARATION

            if (backLeftFoot.position.distanceTo(stepTarget) < velocity.length * 3.0) {
                stepTarget
            } else {
                backLeftFoot.position + (stepTarget - backLeftFoot.position).setLength(velocity.length * 3.0)
            }
        }

        val backRightFootTarget = if (backFootDown == backRightFoot) {
            backRightFoot.position
        } else {
            val stepTarget = backHip.position + Vector2.unitWithRadiansAngle(
                (backHip.parentJoint!!.position - backHip.position).angle() + BACK_FOOT_TARGET_ANGLE.toRadians()
            ) * BACK_FOOT_TARGET_SEPARATION

            if (backRightFoot.position.distanceTo(stepTarget) < velocity.length * 3.0) {
                stepTarget
            } else {
                backRightFoot.position + (stepTarget - backRightFoot.position).setLength(velocity.length * 3.0)
            }
        }

        return mapOf(
            Pair(frontLeftFoot, frontLeftFootTarget),
            Pair(frontRightFoot, frontRightFootTarget),
            Pair(backLeftFoot, backLeftFootTarget),
            Pair(backRightFoot, backRightFootTarget)
        )
    }
}