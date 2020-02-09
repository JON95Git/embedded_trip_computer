#ifndef SCREEN_MONITORVIEW_HPP
#define SCREEN_MONITORVIEW_HPP

#include <gui_generated/screen_monitor_screen/Screen_monitorViewBase.hpp>
#include <gui/screen_monitor_screen/Screen_monitorPresenter.hpp>

#include "../../../../Core/Inc/Gauge.hpp"



class Screen_monitorView : public Screen_monitorViewBase
{
public:
    Screen_monitorView();
    virtual ~Screen_monitorView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void init_gauge(int value);
    virtual void update_gauge(int value);
    virtual void handleTickEvent();

protected:
    Gauge gauge;
private:
    static const uint16_t CANVAS_BUFFER_SIZE = 7200;
    uint8_t canvasBuffer[CANVAS_BUFFER_SIZE];
};

#endif // SCREEN_MONITORVIEW_HPP
