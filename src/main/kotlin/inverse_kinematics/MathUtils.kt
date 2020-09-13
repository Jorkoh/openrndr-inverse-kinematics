package inverse_kinematics

import org.openrndr.math.Vector2
import kotlin.math.sqrt

fun Vector2.setLength(newLength: Double): Vector2 {
    return this * sqrt(newLength * newLength / squaredLength)
}