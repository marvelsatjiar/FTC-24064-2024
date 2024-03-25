package org.firstinspires.ftc.teamcode.control.gainmatrices

data class KalmanGains @JvmOverloads constructor(
        @JvmField var Q: Double = 0.1, // Trust in model
        @JvmField var R: Double = 0.4, // Trust in sensor
)