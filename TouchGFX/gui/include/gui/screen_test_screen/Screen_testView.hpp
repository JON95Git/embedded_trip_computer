#ifndef SCREEN_TESTVIEW_HPP
#define SCREEN_TESTVIEW_HPP

#include <gui_generated/screen_test_screen/Screen_testViewBase.hpp>
#include <gui/screen_test_screen/Screen_testPresenter.hpp>

class Screen_testView : public Screen_testViewBase
{
public:
    Screen_testView();
    virtual ~Screen_testView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void Led();
    virtual void update_wild_card();
    virtual void handleTickEvent();

protected:
};

#endif // SCREEN_TESTVIEW_HPP
