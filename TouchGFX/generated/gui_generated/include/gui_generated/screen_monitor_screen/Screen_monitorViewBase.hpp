/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef SCREEN_MONITORVIEWBASE_HPP
#define SCREEN_MONITORVIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/screen_monitor_screen/Screen_monitorPresenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/Button.hpp>
#include <touchgfx/widgets/TextArea.hpp>

class Screen_monitorViewBase : public touchgfx::View<Screen_monitorPresenter>
{
public:
    Screen_monitorViewBase();
    virtual ~Screen_monitorViewBase() {}
    virtual void setupScreen();

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box box1;
    touchgfx::Button button_back;
    touchgfx::Button button_next;
    touchgfx::TextArea textArea2;

private:

    /*
     * Callback Declarations
     */
    touchgfx::Callback<Screen_monitorViewBase, const touchgfx::AbstractButton&> buttonCallback;

    /*
     * Callback Handler Declarations
     */
    void buttonCallbackHandler(const touchgfx::AbstractButton& src);

};

#endif // SCREEN_MONITORVIEWBASE_HPP