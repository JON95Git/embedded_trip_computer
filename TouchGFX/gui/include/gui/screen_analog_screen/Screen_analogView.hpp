#ifndef SCREEN_ANALOGVIEW_HPP
#define SCREEN_ANALOGVIEW_HPP

#include <gui_generated/screen_analog_screen/Screen_analogViewBase.hpp>
#include <gui/screen_analog_screen/Screen_analogPresenter.hpp>

class Screen_analogView : public Screen_analogViewBase
{
public:
    Screen_analogView();
    virtual ~Screen_analogView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    virtual void update_wild_card();
protected:
};

#endif // SCREEN_ANALOGVIEW_HPP
