package com.scrapmetal.quackerama

import org.firstinspires.ftc.robotcore.external.Telemetry

class SelectionMenu(val telemetry: Telemetry) {
    data class Setting(val caption: String, val options: List<Option>) {
        var currentOptionIndex = 0
    }
    data class Option(val name: String, val value: Any)
    var settings: MutableList<Setting> = mutableListOf()
    var currentSettingIndex = 0

    init {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
    }

    fun addSetting(setting: Setting) {
        settings.add(setting)
    }

    fun refresh(moveUp: Boolean, moveDown: Boolean, switchLeft: Boolean, switchRight: Boolean) {
        // TODO: switch bad index logic to nice iterator stuff?
        when {
            moveUp && currentSettingIndex > 0 -> currentSettingIndex--
            moveDown && currentSettingIndex < settings.lastIndex -> currentSettingIndex++
        }
        settings[currentSettingIndex].apply {
            when {
                switchLeft && currentOptionIndex > 0 -> currentOptionIndex--
                switchLeft && currentOptionIndex == 0 -> currentOptionIndex = options.lastIndex
                switchRight && currentOptionIndex < options.lastIndex -> currentOptionIndex++
                switchRight && currentOptionIndex == options.lastIndex -> currentOptionIndex = 0
            }
        }
        for (setting in settings) {
            if (settings.indexOf(setting) == currentSettingIndex) {
                telemetry.addData("<font color=\"#00FF00\"><big><b><i><u>" + setting.caption + "</u></i></b></big></font>",
                                  setting.options[setting.currentOptionIndex].name)
            } else {
                telemetry.addData(setting.caption, setting.options[setting.currentOptionIndex].name)
            }
        }
    }

    fun slurpSettings(): List<Any> {
        return settings.map { it.options[it.currentOptionIndex].value }
    }
}