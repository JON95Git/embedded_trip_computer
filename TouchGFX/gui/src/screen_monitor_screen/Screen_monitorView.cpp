#include <gui/screen_monitor_screen/Screen_monitorView.hpp>
#include "main.h"

int tickCounter = 0;

Screen_monitorView::Screen_monitorView()
{
	  touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);
}

void Screen_monitorView::setupScreen()
{
    Screen_monitorViewBase::setupScreen();
    // Inicia gauge com valor 0
    init_gauge(0);
    tearDownScreen();
}

void Screen_monitorView::tearDownScreen()
{
    Screen_monitorViewBase::tearDownScreen();
}

void Screen_monitorView::init_gauge(int value)
{
    gauge.setBitmaps(Bitmap(BITMAP_GAUGE_BACKGROUND_ID), Bitmap(BITMAP_GAUGE_NEEDLE_PIN_ID));
    //gauge.setLimits(0, 4096, 240, 480);
    gauge.setLimits(0, 4096, 220, 500);
    gauge.setXY(120,10);
    gauge.setEasingEquation(EasingEquations::linearEaseNone);
    // 18 para animacao eh o limite empirico
    gauge.setAnimationDuration(18);
	gauge.setValue(value);
	add(gauge);
}

void Screen_monitorView::update_gauge(int value)
{
	gauge.setValue(value);
}

void Screen_monitorView::handleTickEvent()
{
	// Atualiza valor do gauge
	tickCounter ++;
	if (tickCounter % 20 == 0){
		update_gauge(g_ADCValue/*/135*/);
	}
	if (tickCounter >= 200){
		tickCounter = 0;
	}
}










