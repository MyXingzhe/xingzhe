#include "wx/wxprec.h"
#include "wx/toolbar.h"
#include "wx/log.h"
#include "wx/image.h"
#include "wx/filedlg.h"
#include "wx/colordlg.h"
#include "wx/srchctrl.h"
#include "wx/collpane.h"



enum MySpecialWidgetStyles
{
    SWS_LOOK_CRAZY = 1,
    SWS_LOOK_SERIOUS = 2,
    SWS_SHOW_BUTTON = 4,
    SWS_DEFAULT_STYLE = (SWS_SHOW_BUTTON|SWS_LOOK_SERIOUS)
};

class OdoPanel   : public wxControl
{
public:
    OdoControl() { Init(); }
    OdoControl(wxWindow *parent,
                    wxWindowID winid,
                    const wxString& label,
                    const wxPoint& pos = wxDefaultPosition,
                    const wxSize& size = wxDefaultSize,
                    long style = SWS_DEFAULT_STYLE,
                    const wxValidator& val = wxDefaultValidator,
                    const wxString& name = "OdoControl")
    {
        Init();
        Create(parent, winid, label, pos, size, style, val, name);
    }
    bool Create(wxWindow *parent,
                wxWindowID winid,
                const wxString& label,
                const wxPoint& pos = wxDefaultPosition,
                const wxSize& size = wxDefaultSize,
                long style = SWS_DEFAULT_STYLE,
                const wxValidator& val = wxDefaultValidator,
                const wxString& name = wxCollapsiblePaneNameStr);
    // accessors...
protected:
    void Init() {
        // init widget's internals...
    }
    virtual wxSize DoGetBestSize() const {
        // we need to calculate and return the best size of the widget...
    }
    void OnPaint(wxPaintEvent&) {
        // draw the widget on a wxDC...
    }
private:
    wxDECLARE_DYNAMIC_CLASS(OdoControl);
    wxDECLARE_EVENT_TABLE();
};


