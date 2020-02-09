#ifndef SCREEN_INITPRESENTER_HPP
#define SCREEN_INITPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Screen_initView;

class Screen_initPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen_initPresenter(Screen_initView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~Screen_initPresenter() {};

private:
    Screen_initPresenter();

    Screen_initView& view;
};


#endif // SCREEN_INITPRESENTER_HPP
