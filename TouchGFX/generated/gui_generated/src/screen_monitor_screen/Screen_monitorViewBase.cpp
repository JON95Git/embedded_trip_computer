/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen_monitor_screen/Screen_monitorViewBase.hpp>
#include <touchgfx/Color.hpp>
#include "BitmapDatabase.hpp"
#include <texts/TextKeysAndLanguages.hpp>

Screen_monitorViewBase::Screen_monitorViewBase() :
    buttonCallback(this, &Screen_monitorViewBase::buttonCallbackHandler)
{

    box1.setPosition(0, 0, 480, 272);
    box1.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));

    button_back.setXY(13, 207);
    button_back.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_ICONS_BACK_ARROW_48_ID), touchgfx::Bitmap(BITMAP_DARK_ICONS_BACK_ARROW_48_ID));
    button_back.setAction(buttonCallback);

    button_next.setXY(442, 207);
    button_next.setBitmaps(touchgfx::Bitmap(BITMAP_DARK_ICONS_NEXT_ARROW_48_ID), touchgfx::Bitmap(BITMAP_DARK_ICONS_NEXT_ARROW_48_ID));
    button_next.setAction(buttonCallback);

    textArea2.setXY(155, 247);
    textArea2.setColor(touchgfx::Color::getColorFrom24BitRGB(252, 252, 252));
    textArea2.setLinespacing(0);
    textArea2.setTypedText(touchgfx::TypedText(T_SINGLEUSEID19));

    add(box1);
    add(button_back);
    add(button_next);
    add(textArea2);
}

void Screen_monitorViewBase::setupScreen()
{

}

void Screen_monitorViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &button_back)
    {
        //Interaction1
        //When button_back clicked change screen to Screen_test
        //Go to Screen_test with screen transition towards West
        application().gotoScreen_testScreenSlideTransitionWest();
    }
    else if (&src == &button_next)
    {
        //Interaction2
        //When button_next clicked change screen to Screen_analog
        //Go to Screen_analog with screen transition towards East
        application().gotoScreen_analogScreenSlideTransitionEast();
    }
}
