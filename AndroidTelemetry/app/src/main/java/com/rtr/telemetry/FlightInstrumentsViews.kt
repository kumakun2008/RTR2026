package com.rtr.telemetry

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View
import java.util.Locale
import kotlin.math.cos
import kotlin.math.sin

class AirspeedIndicatorView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    var airspeed: Float = 0f
        set(value) {
            field = value
            postInvalidate()
        }

    private val bgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#1E293B") // card_dark
        style = Paint.Style.FILL
    }

    private val borderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#3B82F6") // accent_blue
        style = Paint.Style.STROKE
        strokeWidth = 4f
    }

    private val tickPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }

    private val needlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#EF4444") // accent_red
        style = Paint.Style.FILL_AND_STROKE
        strokeWidth = 6f
        strokeCap = Paint.Cap.ROUND
    }

    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 36f
        textAlign = Paint.Align.CENTER
        typeface = Typeface.DEFAULT_BOLD
    }

    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#94A3B8") // text_secondary
        textSize = 28f
        textAlign = Paint.Align.CENTER
        typeface = Typeface.DEFAULT_BOLD
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        val cx = w / 2f
        val cy = h / 2f
        val radius = Math.min(cx, cy) - 10f

        // Draw background
        canvas.drawCircle(cx, cy, radius, bgPaint)
        canvas.drawCircle(cx, cy, radius, borderPaint)

        // Draw scale ticks (0 to 40 m/s, sweep from 135 deg to 405 deg)
        val startAngle = 135f
        val sweepAngle = 270f
        val maxSpeed = 40f
        
        for (i in 0..40 step 5) {
            val angle = startAngle + (i / maxSpeed) * sweepAngle
            val rad = Math.toRadians(angle.toDouble())
            val startX = cx + (radius - 15f) * cos(rad).toFloat()
            val startY = cy + (radius - 15f) * sin(rad).toFloat()
            val endX = cx + (radius - 5f) * cos(rad).toFloat()
            val endY = cy + (radius - 5f) * sin(rad).toFloat()
            canvas.drawLine(startX, startY, endX, endY, tickPaint)
        }

        // Draw needle
        val targetSpeed = Math.min(Math.max(airspeed, 0f), maxSpeed)
        val needleAngle = startAngle + (targetSpeed / maxSpeed) * sweepAngle
        val needleRad = Math.toRadians(needleAngle.toDouble())
        val needleLength = radius - 30f
        val needleX = cx + needleLength * cos(needleRad).toFloat()
        val needleY = cy + needleLength * sin(needleRad).toFloat()
        canvas.drawLine(cx, cy, needleX, needleY, needlePaint)
        canvas.drawCircle(cx, cy, 12f, needlePaint)

        // Draw center digital text with Locale.US
        canvas.drawText(String.format(Locale.US, "%.1f m/s", airspeed), cx, cy + 60f, textPaint)
        
        // Draw title
        canvas.drawText("AIRSPEED", cx, cy - radius / 2f, titlePaint)
    }
}

class AttitudeIndicatorView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    var pitch: Float = 0f
        set(value) {
            field = value
            postInvalidate()
        }

    var roll: Float = 0f
        set(value) {
            field = value
            postInvalidate()
        }

    private val skyPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#38BDF8") // Sky blue
        style = Paint.Style.FILL
    }

    private val groundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#78350F") // Earth brown
        style = Paint.Style.FILL
    }

    private val whitePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }

    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 22f
        textAlign = Paint.Align.CENTER
    }

    private val aircraftPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#F59E0B") // Yellow/Amber
        style = Paint.Style.STROKE
        strokeWidth = 8f
        strokeCap = Paint.Cap.ROUND
    }

    private val borderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#06B6D4") // accent_cyan
        style = Paint.Style.STROKE
        strokeWidth = 4f
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        val cx = w / 2f
        val cy = h / 2f
        val radius = Math.min(cx, cy) - 10f

        // Clip to circular face
        val clipPath = Path().apply {
            addCircle(cx, cy, radius, Path.Direction.CW)
        }
        
        canvas.save()
        canvas.clipPath(clipPath)

        // Rotate and translate for pitch/roll
        canvas.save()
        canvas.rotate(-roll, cx, cy)
        
        // Pitch mapping: 1 degree = 2.5 pixels
        val pitchOffset = pitch * 2.5f
        canvas.translate(0f, pitchOffset)

        // Draw sky and ground
        canvas.drawRect(cx - radius * 2f, cy - radius * 2f, cx + radius * 2f, cy, skyPaint)
        canvas.drawRect(cx - radius * 2f, cy, cx + radius * 2f, cy + radius * 2f, groundPaint)
        canvas.drawLine(cx - radius * 2f, cy, cx + radius * 2f, cy, whitePaint)

        // Draw pitch lines (every 10 degrees)
        for (p in -30..30 step 10) {
            if (p == 0) continue
            val y = cy - p * 2.5f
            val width = if (p % 20 == 0) 60f else 30f
            canvas.drawLine(cx - width, y, cx + width, y, whitePaint)
            canvas.drawText(p.toString(), cx - width - 15f, y + 8f, textPaint)
            canvas.drawText(p.toString(), cx + width + 15f, y + 8f, textPaint)
        }

        canvas.restore() // Restore pitch/roll rotation/translation

        // Draw center reference aircraft symbol (Fixed)
        canvas.drawLine(cx - 50f, cy, cx - 20f, cy, aircraftPaint)
        canvas.drawLine(cx - 20f, cy, cx - 20f, cy + 15f, aircraftPaint)
        canvas.drawLine(cx + 20f, cy, cx + 50f, cy, aircraftPaint)
        canvas.drawLine(cx + 20f, cy, cx + 20f, cy + 15f, aircraftPaint)
        canvas.drawCircle(cx, cy, 6f, Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.parseColor("#F59E0B")
            style = Paint.Style.FILL
        })

        canvas.restore() // Restore circular clip

        // Draw outer border
        canvas.drawCircle(cx, cy, radius, borderPaint)
        
        // Draw digital values text with Locale.US
        val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            textSize = 28f
            textAlign = Paint.Align.CENTER
            typeface = Typeface.DEFAULT_BOLD
        }
        canvas.drawText(String.format(Locale.US, "P:%.1f° R:%.1f°", pitch, roll), cx, cy + radius - 30f, labelPaint)
    }
}

class AltimeterIndicatorView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    var altitude: Float = 0f
        set(value) {
            field = value
            postInvalidate()
        }

    private val bgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#1E293B") // card_dark
        style = Paint.Style.FILL
    }

    private val borderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#10B981") // accent_green
        style = Paint.Style.STROKE
        strokeWidth = 4f
    }

    private val tickPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }

    private val needlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#EF4444") // accent_red
        style = Paint.Style.FILL_AND_STROKE
        strokeWidth = 6f
        strokeCap = Paint.Cap.ROUND
    }

    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 36f
        textAlign = Paint.Align.CENTER
        typeface = Typeface.DEFAULT_BOLD
    }

    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#94A3B8") // text_secondary
        textSize = 28f
        textAlign = Paint.Align.CENTER
        typeface = Typeface.DEFAULT_BOLD
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        val cx = w / 2f
        val cy = h / 2f
        val radius = Math.min(cx, cy) - 10f

        // Draw background
        canvas.drawCircle(cx, cy, radius, bgPaint)
        canvas.drawCircle(cx, cy, radius, borderPaint)

        // Draw scale ticks (0 to 10 m, full circle sweep)
        val maxAlt = 10f
        for (i in 0..9) {
            val angle = 270f + (i / maxAlt) * 360f
            val rad = Math.toRadians(angle.toDouble())
            val startX = cx + (radius - 15f) * cos(rad).toFloat()
            val startY = cy + (radius - 15f) * sin(rad).toFloat()
            val endX = cx + (radius - 5f) * cos(rad).toFloat()
            val endY = cy + (radius - 5f) * sin(rad).toFloat()
            canvas.drawLine(startX, startY, endX, endY, tickPaint)
            
            // Draw scale numbers
            val textX = cx + (radius - 35f) * cos(rad).toFloat()
            val textY = cy + (radius - 35f) * sin(rad).toFloat() + 10f
            canvas.drawText(i.toString(), textX, textY, Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color = Color.WHITE
                textSize = 20f
                textAlign = Paint.Align.CENTER
            })
        }

        // Draw needle
        val needleAngle = 270f + (altitude / maxAlt) * 360f
        val needleRad = Math.toRadians(needleAngle.toDouble())
        val needleLength = radius - 40f
        val needleX = cx + needleLength * cos(needleRad).toFloat()
        val needleY = cy + needleLength * sin(needleRad).toFloat()
        canvas.drawLine(cx, cy, needleX, needleY, needlePaint)
        canvas.drawCircle(cx, cy, 12f, needlePaint)

        // Draw center digital text with Locale.US
        canvas.drawText(String.format(Locale.US, "%.2f m", altitude), cx, cy + 60f, textPaint)
        
        // Draw title
        canvas.drawText("ALTITUDE", cx, cy - radius / 2f, titlePaint)
    }
}

class HeadingIndicatorView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    var heading: Float = 0f
        set(value) {
            field = value
            postInvalidate()
        }

    private val bgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#1E293B") // card_dark
        style = Paint.Style.FILL
    }

    private val borderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#EC4899") // Pink/Magenta
        style = Paint.Style.STROKE
        strokeWidth = 4f
    }

    private val cardPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }

    private val pointerPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#EF4444") // accent_red
        style = Paint.Style.FILL
    }

    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 36f
        textAlign = Paint.Align.CENTER
        typeface = Typeface.DEFAULT_BOLD
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        val cx = w / 2f
        val cy = h / 2f
        val radius = Math.min(cx, cy) - 10f

        // Draw background
        canvas.drawCircle(cx, cy, radius, bgPaint)
        canvas.drawCircle(cx, cy, radius, borderPaint)

        // Save canvas state to rotate the compass card
        canvas.save()
        canvas.rotate(-heading, cx, cy)

        // Draw compass markings
        val cardRadius = radius - 15f
        canvas.drawCircle(cx, cy, cardRadius, cardPaint)

        val cardinalLabels = mapOf(
            0 to "N",
            90 to "E",
            180 to "S",
            270 to "W"
        )

        for (angle in 0..350 step 30) {
            val rad = Math.toRadians((angle - 90).toDouble())
            val startX = cx + (cardRadius - 15f) * cos(rad).toFloat()
            val startY = cy + (cardRadius - 15f) * sin(rad).toFloat()
            val endX = cx + cardRadius * cos(rad).toFloat()
            val endY = cy + cardRadius * sin(rad).toFloat()
            canvas.drawLine(startX, startY, endX, endY, cardPaint)

            // Draw label
            val label = cardinalLabels[angle] ?: angle.toString()
            val textX = cx + (cardRadius - 35f) * cos(rad).toFloat()
            val textY = cy + (cardRadius - 35f) * sin(rad).toFloat() + 10f
            canvas.drawText(label, textX, textY, Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color = if (label in cardinalLabels.values) Color.parseColor("#F59E0B") else Color.WHITE
                textSize = if (label in cardinalLabels.values) 24f else 18f
                textAlign = Paint.Align.CENTER
                typeface = Typeface.DEFAULT_BOLD
            })
        }

        canvas.restore() // Restore original rotation

        // Draw fixed pointer at the top
        val pointerPath = Path().apply {
            moveTo(cx, cy - radius + 20f)
            lineTo(cx - 10f, cy - radius + 40f)
            lineTo(cx + 10f, cy - radius + 40f)
            close()
        }
        canvas.drawPath(pointerPath, pointerPaint)

        // Draw digital values text with Locale.US
        canvas.drawText(String.format(Locale.US, "%03d°", (heading.toInt() + 360) % 360), cx, cy + 15f, textPaint)
    }
}
