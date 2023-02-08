package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils

abstract class FourTrackingWheelLocalizer (
        wheelPoses: List<Pose2d>
) : Localizer {
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
    private var lastWheelPositions = emptyList<Double>()

    private val forwardSolver1: DecompositionSolver
    private val forwardSolver2: DecompositionSolver
    private val forwardSolver3: DecompositionSolver
    private val forwardSolver4: DecompositionSolver

    init {
        require(wheelPoses.size == 4) { "4 wheel positions must be provided" }

        val inverseMatrix1 = Array2DRowRealMatrix(3, 3)
        val w1 =  intArrayOf(0, 1, 2)
        for (i in 0..2) {
            val orientationVector = wheelPoses[w1[i]].headingVec()
            val positionVector = wheelPoses[w1[i]].vec()
            inverseMatrix1.setEntry(i, 0, orientationVector.x)
            inverseMatrix1.setEntry(i, 1, orientationVector.y)
            inverseMatrix1.setEntry(
                    i,
                    2,
                    positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            )
        }
        forwardSolver1 = LUDecomposition(inverseMatrix1).solver
        require(forwardSolver1.isNonSingular) { "The specified configuration cannot support full localization" }

        val inverseMatrix2 = Array2DRowRealMatrix(3, 3)
        val w2 =  intArrayOf(0, 1, 3)
        for (i in 0..2) {
            val orientationVector = wheelPoses[w2[i]].headingVec()
            val positionVector = wheelPoses[w2[i]].vec()
            inverseMatrix2.setEntry(i, 0, orientationVector.x)
            inverseMatrix2.setEntry(i, 1, orientationVector.y)
            inverseMatrix2.setEntry(
                i,
                2,
                positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            )
        }

        forwardSolver2 = LUDecomposition(inverseMatrix2).solver
        require(forwardSolver2.isNonSingular) { "The specified configuration cannot support full localization" }



        val inverseMatrix3 = Array2DRowRealMatrix(3, 3)
        val w3 =  intArrayOf(0, 2, 3) //change to 2
        for (i in 0..2) {
            val orientationVector = wheelPoses[w3[i]].headingVec()
            val positionVector = wheelPoses[w3[i]].vec()
            inverseMatrix3.setEntry(i, 0, orientationVector.x)
            inverseMatrix3.setEntry(i, 1, orientationVector.y)
            inverseMatrix3.setEntry(
                i,
                2,
                positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            )
        }
        forwardSolver3 = LUDecomposition(inverseMatrix3).solver
        require(forwardSolver1.isNonSingular) { "The specified configuration cannot support full localization" }

        val inverseMatrix4 = Array2DRowRealMatrix(3, 3)
        val w4 =  intArrayOf(1, 2, 3) //change to 3
        for (i in 0..2) {
            val orientationVector = wheelPoses[w4[i]].headingVec()
            val positionVector = wheelPoses[w4[i]].vec()
            inverseMatrix4.setEntry(i, 0, orientationVector.x)
            inverseMatrix4.setEntry(i, 1, orientationVector.y)
            inverseMatrix4.setEntry(
                i,
                2,
                positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            )
        }
        forwardSolver4 = LUDecomposition(inverseMatrix4).solver
        require(forwardSolver1.isNonSingular) { "The specified configuration cannot support full localization" }
    }

    private fun calculatePoseDelta(wheelDeltas: List<Double>): Pose2d {
        var wd = wheelDeltas.toDoubleArray();
        val rawPoseDelta1 = forwardSolver1.solve(
                MatrixUtils.createRealMatrix(
                        arrayOf(doubleArrayOf(wd[0], wd[1], wd[2]))
                ).transpose()
        )
        val rawPoseDelta2 = forwardSolver2.solve(
            MatrixUtils.createRealMatrix(
                arrayOf(doubleArrayOf(wd[0], wd[1], wd[3]))
            ).transpose()
        )
        val rawPoseDelta3 = forwardSolver3.solve(
            MatrixUtils.createRealMatrix(
                arrayOf(doubleArrayOf(wd[0], wd[2], wd[3])) //??
            ).transpose()
        )
        val rawPoseDelta4 = forwardSolver4.solve(
            MatrixUtils.createRealMatrix(
                arrayOf(doubleArrayOf(wd[1], wd[2], wd[3]))
            ).transpose()
        )

        return Pose2d(
            (rawPoseDelta1.getEntry(0, 0) + rawPoseDelta2.getEntry(0, 0)
                    + rawPoseDelta3.getEntry(0, 0) + rawPoseDelta4.getEntry(0, 0))/4,
            (rawPoseDelta1.getEntry(1, 0) + rawPoseDelta2.getEntry(1, 0)
                    + rawPoseDelta3.getEntry(1, 0) + rawPoseDelta4.getEntry(1, 0))/4,
            (rawPoseDelta1.getEntry(2, 0) + rawPoseDelta2.getEntry(2, 0)
                    + rawPoseDelta3.getEntry(2, 0) + rawPoseDelta4.getEntry(2, 0))/4,
        )
    }

    override fun update() {
        val wheelPositions = getWheelPositions()
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
            val robotPoseDelta = calculatePoseDelta(wheelDeltas)
            _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, robotPoseDelta)
        }

        val wheelVelocities = getWheelVelocities()
        if (wheelVelocities != null) {
            poseVelocity = calculatePoseDelta(wheelVelocities)
        }

        lastWheelPositions = wheelPositions
    }

    /**
     * Returns the positions of the tracking wheels in the desired distance units (not encoder counts!)
     */
    abstract fun getWheelPositions(): List<Double>

    /**
     * Returns the velocities of the tracking wheels in the desired distance units (not encoder counts!)
     */
    open fun getWheelVelocities(): List<Double>? = null
}