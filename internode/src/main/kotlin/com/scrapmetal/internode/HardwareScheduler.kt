package com.scrapmetal.internode

/**
 * Schedule hardware reads and writes
 */
object HardwareScheduler {
    /**
     * Keep track of how many loops have run, for update periods
     */
    var loopCount = 0

    /**
     * Perform all hardware reads
     *
     * This should not take significant time due to bulkreads
     */
    fun read() = NodeBroker.subsystems.forEach { it.read() }

    /**
     * Perform all scheduled hardware writes
     *
     * Hardware write count should be limited to 8 at most
     */
    fun write() {
        loopCount++
        NodeBroker.subsystems.forEach {
            if (loopCount % it.updatePeriod == 0) {
                it.write()
            }
        }
    }
}