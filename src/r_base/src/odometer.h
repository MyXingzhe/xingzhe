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

	void SetSpeed(float speed);
	void SetDistance(int distance);

private:
    void OnSize(wxSizeEvent& event);
    void OnPaint(wxPaintEvent& event);;

	float m_speed = 18.0;
	int m_distance = 0;

};


#endif
