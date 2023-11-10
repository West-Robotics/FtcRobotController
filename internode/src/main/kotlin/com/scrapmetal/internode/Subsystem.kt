package com.scrapmetal.internode

import com.scrapmetal.internode.Node

/**
 * A special node who's only job is to interface with hardware
 */
interface Subsystem : Node {
    /**
     * @property updatePeriod the number of loops to wait between hardware writes
     */
    var updatePeriod: Int

    /**
     * Perform a hardware read
     */
    fun read()

    /**
     * Perform a hardware write
     */
    fun write()
}