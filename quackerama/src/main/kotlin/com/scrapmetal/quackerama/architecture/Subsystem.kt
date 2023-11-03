package com.scrapmetal.quackerama.architecture

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