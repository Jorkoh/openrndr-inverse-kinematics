package gecko

import org.openrndr.math.Vector2

object World {
    const val WIDTH = 1600.0
    const val HEIGHT = 900.0

    val gecko = Gecko.spawnGecko()

    fun update(objective: Vector2) {
        gecko.update(objective)
    }
}