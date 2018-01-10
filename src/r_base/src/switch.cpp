
#include <math.h>
#include <wx/wx.h>

#include "switch.h"

SwitchPanel::SwitchPanel(wxPanel *parent, int id)
      : wxPanel(parent, id, wxDefaultPosition, wxSize(30, 30), wxSUNKEN_BORDER)
{
 
  m_state = 0;

  Connect(wxEVT_PAINT, wxPaintEventHandler(SwitchPanel::OnPaint));
  Connect(wxEVT_SIZE, wxSizeEventHandler(SwitchPanel::OnSize));

  this->Connect(wxEVT_LEFT_UP, wxMouseEventHandler(SwitchPanel::ChangeState), NULL, this);

}

void SwitchPanel::OnPaint(wxPaintEvent& event)
{
  int border_width = 2;
  wxPaintDC dc(this);
  wxSize size = GetSize();
  int width = size.GetWidth();
  int height = size.GetHeight();

  wxPoint center(width/4, height/2);
  wxCoord r = ((width<height)?(width/4 - border_width):(height/3 - border_width));


  if(m_state==1)
  {
    dc.SetPen(wxPen(wxColour(wxT("#37c435")), border_width, wxSOLID));
    dc.SetBrush(wxBrush(wxColour(wxT("#37c435"))));
    dc.DrawRoundedRectangle(center.x-r, center.y-r, r*4, r*2, r);

    dc.SetBrush(wxBrush(wxColour(wxT("#F9F9F9"))));
    center.x += r*2;
  }
  else
  {
    dc.SetPen(wxPen(wxColour(wxT("#B9B9B9")), border_width, wxSOLID));
    dc.SetBrush(wxBrush(wxColour(wxT("#B9B9B9"))));
    dc.DrawRoundedRectangle(center.x-r, center.y-r, r*4, r*2, r);

    dc.SetBrush(wxBrush(wxColour(wxT("#F9F9F9"))));
  }
  dc.DrawCircle(center, r);
}

void SwitchPanel::OnSize(wxSizeEvent& event)
{
  Refresh(); 
}

void SwitchPanel::ChangeState(wxMouseEvent& event)
{
  if(m_state==0)
    m_state=1;
  else
    m_state=0;

  Refresh();
}

int SwitchPanel::GetState()
{
  return m_state;
}