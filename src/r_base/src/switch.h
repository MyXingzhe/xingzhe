#ifndef __SWITCH_PANEL__
#define __SWITCH_PANEL__

#include <wx/bitmap.h>
#include <wx/checkbox.h>
#include <wx/event.h>
#include <wx/gdicmn.h>
#include <wx/slider.h>
#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/window.h>
#include <wx/panel.h>


class SwitchPanel : public wxPanel
{
public:
    SwitchPanel(wxPanel *parent, int id );
    int GetState();

private:
    int m_state;

    void OnSize(wxSizeEvent& event);
    void OnPaint(wxPaintEvent& event);
    void ChangeState(wxMouseEvent& event);
};


#endif
