package com.scrapmetal.quackerama.architecture

interface Node {
    /**
     * Topic map to be published to the master map
     */
    val topics: MutableMap<String, Any>

    /**
     * Updates the node with given timestemp [dt] in ms
     *
     * Call once per scheduled loop
     * Publish and subscribe here
     * Read from subscriptions, perform tasks, publish
     */
    fun update(dt: Double)

    fun publish(string: String, data: Any) {
        topics[string] = data
    }
}