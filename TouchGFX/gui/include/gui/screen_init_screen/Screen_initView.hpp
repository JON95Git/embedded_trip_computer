#ifndef SCREEN_INITVIEW_HPP
#define SCREEN_INITVIEW_HPP

#include <gui_generated/screen_init_screen/Screen_initViewBase.hpp>
#include <gui/screen_init_screen/Screen_initPresenter.hpp>

class Screen_initView : public Screen_initViewBase
{
public:
    Screen_initView();
    virtual ~Screen_initView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SCREEN_INITVIEW_HPP
