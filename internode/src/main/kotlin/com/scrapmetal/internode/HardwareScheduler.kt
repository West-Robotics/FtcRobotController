package com.scrapmetal.internode

/**
 * Schedule hardware reads and writes
 *
 * TODO: specify scheduling inside here instead of delegating the job to subsystems
 *   they don't have to care about priority, this does
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