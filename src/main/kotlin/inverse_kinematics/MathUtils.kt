package inverse_kinematics

import org.openrndr.math.Vector2
import kotlin.math.*

fun Double.toDegrees() = Math.toDegrees(this)

fun Double.toRadians() = Math.toRadians(this)

fun Vector2.angle() = atan2(y, x)

fun Vector2.setAngle(newAngle: Double): Vector2 {
    return Vector2(length * cos(newAngle), length * sin(newAngle))
}

fun Vector2.setLength(newLength: Double): Vector2 {
    return this * sqrt(newLength * newLength / squaredLength)
}


fun Vector2.Companion.unitWithAngle(angle: Double): Vector2 {
    return Vector2(cos(angle), sin(angle))
}

fun Vector2.angle(secondVector: Vector2): Double {
    return acos(this.dot(secondVector) / sqrt(squaredLength * secondVector.squaredLength))
}

fun Vector2.determinant(secondVector: Vector2): Double {
    return x * secondVector.y - y * secondVector.x
}

fun Vector2.clockwiseAngle(secondVector: Vector2): Double {
    return atan2(this.determinant(secondVector), this.dot(secondVector)) + PI
}

// https://math.stackexchange.com/a/1649850
fun Double.angleDifference(secondRadiansAngle: Double): Double {
    return (this - secondRadiansAngle + 9.42478) % 6.28319 - 3.14159
}

fun Vector2.clampAngleDifference(secondVector: Vector2, minAngleDiff: Double, maxAngleDiff: Double): Vector2 {
    val angleDiff = (-secondVector).clockwiseAngle(this)
    return when {
        angleDiff > maxAngleDiff -> Vector2.unitWithAngle(secondVector.angle() + maxAngleDiff)
        angleDiff < minAngleDiff -> Vector2.unitWithAngle(secondVector.angle() + minAngleDiff)
        else -> this
    }
}