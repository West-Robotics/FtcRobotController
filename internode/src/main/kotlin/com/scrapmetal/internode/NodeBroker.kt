package com.scrapmetal.internode

import kotlin.time.DurationUnit
import kotlin.time.TimeSource

/**
 * Manage all node updates and hardware scheduling
 */
// TODO: add hardware priorities
//  potentially split hardware scheduler into a different object
object NodeBroker {
    /**
     * Registered nodes
     */
    private var nodes: List<Node> = emptyList()

    /**
     * Registered subsystems
     */
    var subsystems: List<Subsystem> = emptyList()
        private set

    /**
     * Master topic map that all nodes read from
     */
    var topics: MutableMap<String, Any> = mutableMapOf()
        private set
    private val timeSource = TimeSource.Monotonic
    private var currentMark: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()
    private var lastMark: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()
    private var dt = 0.0 // in ms

    /**
     * Register nodes with the broker. ORDER MATTERS
     *
     * Register in dependency order to reduce latency between updates of dependent nodes
     */
    fun registerNodes(vararg nodes: Node) {
        NodeBroker.nodes = nodes.toList()
        nodes.forEach { topics += it.topics }
    }

    /**
     * Register subsystems with the broker. Follow same ordering advice as [registerNodes]
     */
    fun registerSubsystems(vararg subsystems: Subsystem) {
        NodeBroker.subsystems = subsystems.toList()
    }

    /**
     * Let an individual node update and publish their topics
     */
    fun updateNode(node: Node) {
        node.update(dt)
        topics += node.topics
    }

    /**
     * Updates all scheduled nodes. Run once per loop
     */
    fun update() {
        currentMark = timeSource.markNow()
        dt = (currentMark - lastMark).toDouble(DurationUnit.MILLISECONDS)
        lastMark = currentMark
        HardwareScheduler.read()
        nodes.forEach { updateNode(it) }
        HardwareScheduler.write()
    }
}