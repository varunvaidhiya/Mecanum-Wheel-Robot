package com.varunvaidhiya.robotcontrol.ui.views

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class VirtualJoystickView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var centerX = 0f
    private var centerY = 0f
    private var baseRadius = 0f
    private var hatRadius = 0f
    
    private var hatX = 0f
    private var hatY = 0f
    
    private var isTouched = false
    
    // Config
    private val deadzone = 0.1f // 10% deadzone
    
    // Public output
    var onJoystickMoved: ((x: Float, y: Float) -> Unit)? = null

    private val basePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.LTGRAY
        style = Paint.Style.STROKE
        strokeWidth = 5f
        alpha = 100
    }

    private val hatPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#2196F3") // Primary Blue
        style = Paint.Style.FILL
        alpha = 200
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        centerX = w / 2f
        centerY = h / 2f
        baseRadius = min(w, h) / 2.5f
        hatRadius = baseRadius / 3f
        
        resetJoystick()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        
        // Draw Base
        canvas.drawCircle(centerX, centerY, baseRadius, basePaint)
        
        // Draw Hat
        canvas.drawCircle(hatX, hatY, hatRadius, hatPaint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.action) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                isTouched = true
                val displacementX = event.x - centerX
                val displacementY = event.y - centerY
                val distance = sqrt(displacementX.pow(2) + displacementY.pow(2))
                
                if (distance < baseRadius) {
                    hatX = event.x
                    hatY = event.y
                } else {
                    // Capping logic
                    val ratio = baseRadius / distance
                    hatX = centerX + displacementX * ratio
                    hatY = centerY + displacementY * ratio
                }
                
                notifyListener()
                invalidate()
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                isTouched = false
                resetJoystick()
                notifyListener()
                invalidate()
            }
        }
        return true
    }

    private fun resetJoystick() {
        hatX = centerX
        hatY = centerY
    }

    private fun notifyListener() {
        // Calculate normalized values (-1.0 to 1.0)
        // Y is inverted because screen Y grows downwards, but robot Y+ is forward/up
        var normX = (hatX - centerX) / baseRadius
        var normY = -(hatY - centerY) / baseRadius
        
        // Deadzone application
        val magnitude = sqrt(normX.pow(2) + normY.pow(2))
        
        if (magnitude < deadzone) {
            normX = 0f
            normY = 0f
        }
        
        onJoystickMoved?.invoke(normX, normY)
    }
}
