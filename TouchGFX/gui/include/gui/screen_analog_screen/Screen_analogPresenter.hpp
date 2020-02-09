#ifndef SCREEN_ANALOGPRESENTER_HPP
#define SCREEN_ANALOGPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Screen_analogView;

class Screen_analogPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen_analogPresenter(Screen_analogView& v);

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

    virtual ~Screen_analogPresenter() {};

private:
    Screen_analogPresenter();

    Screen_analogView& view;
};

#endif // SCREEN_ANALOGPRESENTER_HPP
