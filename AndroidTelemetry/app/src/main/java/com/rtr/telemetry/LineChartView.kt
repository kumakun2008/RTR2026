package com.rtr.telemetry

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.LinearGradient
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Shader
import android.util.AttributeSet
import android.view.View

/**
 * A beautiful, high-performance custom Line Chart View that plots real-time telemetry points.
 */
class LineChartView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private val dataPoints = ArrayList<Float>()
    private val maxPoints = 50

    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#1E293B")
        strokeWidth = 2f
        style = Paint.Style.STROKE
    }

    private val linePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#06B6D4") // Cyan primary
        strokeWidth = 6f
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }

    private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }

    private val path = Path()
    private val fillPath = Path()

    fun addDataPoint(value: Float) {
        dataPoints.add(value)
        if (dataPoints.size > maxPoints) {
            dataPoints.removeAt(0)
        }
        invalidate() // Request redraw
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (width == 0 || height == 0) return

        drawGrid(canvas)
        drawChart(canvas)
    }

    private fun drawGrid(canvas: Canvas) {
        val rows = 4
        val cols = 8
        val rowHeight = height.toFloat() / rows
        val colWidth = width.toFloat() / cols

        // Draw horizontal grid lines
        for (i in 1 until rows) {
            val y = i * rowHeight
            canvas.drawLine(0f, y, width.toFloat(), y, gridPaint)
        }

        // Draw vertical grid lines
        for (i in 1 until cols) {
            val x = i * colWidth
            canvas.drawLine(x, 0f, x, height.toFloat(), gridPaint)
        }
    }

    private fun drawChart(canvas: Canvas) {
        if (dataPoints.isEmpty()) return

        path.reset()
        fillPath.reset()

        val size = dataPoints.size
        val xStep = width.toFloat() / (maxPoints - 1).coerceAtLeast(1)

        var minVal = dataPoints.minOrNull() ?: 0f
        var maxVal = dataPoints.maxOrNull() ?: 10f
        if (maxVal - minVal < 1.0f) {
            maxVal += 1.0f
            minVal -= 1.0f
        }
        val range = maxVal - minVal

        val pointsToDraw = ArrayList<Pair<Float, Float>>()
        for (i in 0 until size) {
            val rawX = i * xStep
            val normalizedY = (dataPoints[i] - minVal) / range
            val rawY = height.toFloat() - (normalizedY * (height - 40) + 20)
            pointsToDraw.add(Pair(rawX, rawY))
        }

        path.moveTo(pointsToDraw[0].first, pointsToDraw[0].second)
        fillPath.moveTo(pointsToDraw[0].first, height.toFloat())
        fillPath.lineTo(pointsToDraw[0].first, pointsToDraw[0].second)

        // Draw cubic curves for smooth rendering
        for (i in 1 until pointsToDraw.size) {
            val prev = pointsToDraw[i - 1]
            val curr = pointsToDraw[i]
            val controlX1 = (prev.first + curr.first) / 2
            path.cubicTo(controlX1, prev.second, controlX1, curr.second, curr.first, curr.second)
            fillPath.cubicTo(controlX1, prev.second, controlX1, curr.second, curr.first, curr.second)
        }

        fillPath.lineTo(pointsToDraw.last().first, height.toFloat())
        fillPath.close()

        // Create elegant gradient shader for chart fill
        val fillGradient = LinearGradient(
            0f, 0f, 0f, height.toFloat(),
            Color.parseColor("#8006B6D4"), Color.parseColor("#0006B6D4"),
            Shader.TileMode.CLAMP
        )
        fillPaint.shader = fillGradient

        // Draw fill and stroke paths
        canvas.drawPath(fillPath, fillPaint)
        canvas.drawPath(path, linePaint)
    }
}
