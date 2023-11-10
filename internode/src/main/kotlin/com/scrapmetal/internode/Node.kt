package com.scrapmetal.internode

import kotlin.reflect.KClass

interface Node {
    /**
     * Topic map to be published to the master map
     */
    val topics: MutableMap<String, Any>

    /**
     * Call once per scheduled loop
     *
     * Publish and subscribe here
     * Read from subscriptions, perform tasks, publish
     *
     * @param dt time step between loops in milliseconds
     */
    fun update(dt: Double)

    fun publish(topic: String, data: Any) {
        topics[topic] = data
    }
}

inline fun <reified T> Node.receive(topic: String) = NodeBroker.topics[topic] as T