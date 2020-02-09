#ifndef SCREEN_MONITORPRESENTER_HPP
#define SCREEN_MONITORPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Screen_monitorView;

class Screen_monitorPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen_monitorPresenter(Screen_monitorView& v);

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

    virtual ~Screen_monitorPresenter() {};

private:
    Screen_monitorPresenter();

    Screen_monitorView& view;
};

#endif // SCREEN_MONITORPRESENTER_HPP
