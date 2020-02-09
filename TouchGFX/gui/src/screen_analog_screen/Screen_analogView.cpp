#include <gui/screen_analog_screen/Screen_analogView.hpp>
#include "stm32746g_discovery.h"
#include "main.h"

Screen_analogView::Screen_analogView()
{

}

void Screen_analogView::setupScreen()
{
    Screen_analogViewBase::setupScreen();
}

void Screen_analogView::tearDownScreen()
{
    Screen_analogViewBase::tearDownScreen();
}

void Screen_analogView::update_wild_card()
{
	Unicode::snprintfFloat(textArea_batBuffer, 6, "%.1f", (bat_adc/341.3));
	//textArea_bat.resizeToCurrentText();
	textArea_bat.invalidate();

	Unicode::snprintf(textArea_rpmBuffer, 7, "%d", (rpm_adc));
	//textArea_rpm.resizeToCurrentText();
	textArea_rpm.invalidate();


	Unicode::snprintf(textArea_fuelBuffer, 3, "%d", (g_ADCValue/41));
	//textArea_fuel.resizeToCurrentText();
	textArea_fuel.invalidate();

	Unicode::snprintf(textArea_oilBuffer, 3, "%d", (oil_adc/41));
	//textArea_oil.resizeToCurrentText();
	textArea_oil.invalidate();
}
void Screen_analogView::handleTickEvent()
{
	update_wild_card();
}
