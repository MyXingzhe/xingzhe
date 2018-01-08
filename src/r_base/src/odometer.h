#ifndef __ODO_METER__
#define __ODO_METER__

#include <wx/bitmap.h>
#include <wx/checkbox.h>
#include <wx/event.h>
#include <wx/gdicmn.h>
#include <wx/slider.h>
#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/window.h>
#include <wx/panel.h>



class OdoPanel : public wxPanel
{
public:
    OdoPanel(wxPanel *parent, int id );

    wxPanel *m_parent;


    void OnSize(wxSizeEvent& event);
    void OnPaint(wxPaintEvent& event);  

};

class OdoMeter : public wxFrame
{
public:
    OdoMeter(const wxString& title);

    void OnScroll(wxScrollEvent& event);
    int GetCurWidth();


    wxSlider *m_slider;
    OdoPanel *m_wid;

    int cur_width;

};


#endif
