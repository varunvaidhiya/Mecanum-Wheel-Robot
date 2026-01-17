package com.varunvaidhiya.robotcontrol.ui.views

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Rect
import android.util.AttributeSet
import android.view.View
import kotlinx.coroutines.*
import timber.log.Timber
import java.io.BufferedInputStream
import java.net.HttpURLConnection
import java.net.URL

/**
 * Custom View to display MJPEG stream from ROS web_video_server
 */
class MjpegView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private var streamUrl: String? = null
    private var isStreaming = false
    private var streamJob: Job? = null
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())
    
    private var currentBitmap: Bitmap? = null
    private val srcRect = Rect()
    private val dstRect = Rect()
    private val paint = Paint(Paint.ANTI_ALIAS_FLAG)
    
    // UI state
    private var showOverlay = true
    private val overlayPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 40f
        textAlign = Paint.Align.CENTER
    }

    fun startStream(url: String) {
        if (isStreaming && url == streamUrl) return
        
        stopStream()
        streamUrl = url
        isStreaming = true
        
        streamJob = scope.launch {
            try {
                Timber.d("Starting MJPEG stream: $url")
                val connection = URL(url).openConnection() as HttpURLConnection
                connection.connectTimeout = 5000
                connection.readTimeout = 5000
                connection.doInput = true
                connection.connect()

                val inputStream = BufferedInputStream(connection.inputStream)
                
                while (isActive) {
                    val frame = readMjpegFrame(inputStream)
                    if (frame != null) {
                        currentBitmap = frame
                        postInvalidate()
                    }
                }
            } catch (e: Exception) {
                Timber.e(e, "MJPEG Stream error")
            }
        }
    }

    fun stopStream() {
        isStreaming = false
        streamJob?.cancel()
        streamJob = null
        currentBitmap = null
        invalidate()
    }

    private fun readMjpegFrame(bis: BufferedInputStream): Bitmap? {
        // Simplified MJPEG parsing logic
        // In production, robust boundary detection is needed
        // This expects standard MJPEG boundaries
        // TODO: Implement robust content-length parsing
        // For now, allow BitmapFactory to attempt decoding stream
        // Note: This is blocking and naive, but often "just works" for simple streams
        // If it fails, we need manual byte-scanning for 0xFF 0xD8 (SOI) and 0xFF 0xD9 (EOI)
        
        return try {
             BitmapFactory.decodeStream(bis)
        } catch (e: Exception) {
            null
        }
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        
        canvas.drawColor(Color.BLACK)
        
        currentBitmap?.let { bmp ->
            srcRect.set(0, 0, bmp.width, bmp.height)
            
            // Center crop/fit logic
            val viewWidth = width.toFloat()
            val viewHeight = height.toFloat()
            val scale = minOf(viewWidth / bmp.width, viewHeight / bmp.height)
            
            val scaledWidth = bmp.width * scale
            val scaledHeight = bmp.height * scale
            
            val left = (viewWidth - scaledWidth) / 2
            val top = (viewHeight - scaledHeight) / 2
            
            dstRect.set(left.toInt(), top.toInt(), (left + scaledWidth).toInt(), (top + scaledHeight).toInt())
            
            canvas.drawBitmap(bmp, srcRect, dstRect, paint)
        } ?: run {
            // Draw placeholder text
            canvas.drawText("No Signal", width / 2f, height / 2f, overlayPaint)
        }
    }
    
    override fun onDetachedFromWindow() {
        super.onDetachedFromWindow()
        stopStream()
    }
}
