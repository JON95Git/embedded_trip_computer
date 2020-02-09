#include <gui/screen_test_screen/Screen_testView.hpp>
#include "stm32746g_discovery.h"
#include "main.h"
#include "stm32f7xx_hal.h"

Screen_testView::Screen_testView()
{

}

void Screen_testView::setupScreen()
{
    Screen_testViewBase::setupScreen();
}

void Screen_testView::tearDownScreen()
{
    Screen_testViewBase::tearDownScreen();
}
void Screen_testView::Led()
{
	BSP_LED_Toggle(LED_GREEN);
	HAL_GPIO_TogglePin(ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin);
//#define ARDUINO_D2_Pin GPIO_PIN_6
//#define ARDUINO_D2_GPIO_Port GPIOG
    HAL_Delay(50);
}

void Screen_testView::update_wild_card()
{
	Unicode::snprintfFloat(textArea1Buffer, 7, "%.2f", (g_ADCValue/1242.2));
    //textArea1.resizeToCurrentText();
    textArea1.invalidate();
    circleProgress1.setValue(g_ADCValue / 42);
}
void Screen_testView::handleTickEvent()
{
	update_wild_card();
}
