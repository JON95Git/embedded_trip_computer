#ifndef SCREEN_TESTPRESENTER_HPP
#define SCREEN_TESTPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Screen_testView;

class Screen_testPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen_testPresenter(Screen_testView& v);

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

    virtual ~Screen_testPresenter() {};

private:
    Screen_testPresenter();

    Screen_testView& view;
};


#endif // SCREEN_TESTPRESENTER_HPP
