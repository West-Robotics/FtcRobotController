package com.scrapmetal.quackerama.architecture

interface Node {
    /**
     * Topic map to be published to the master map
     */
    val topics: MutableMap<String, Any>

    /**
     * Call once per scheduled loop
     *
     * Publish and subscribe here
     *
     * @param dt time step between loops in milliseconds
     */
    fun update(dt: Double)
}