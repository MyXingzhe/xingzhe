#include <wx/wx.h>
#include "odometer.h"
int num[] = { 75, 150, 225, 300, 375, 450, 525, 600, 675 };
int asize = sizeof(num)/sizeof(num[1]);
int ID_SLIDER = 1;

OdoPanel::OdoPanel(wxPanel *parent, int id)
      : wxPanel(parent, id, wxDefaultPosition, wxSize(-1, 30), wxSUNKEN_BORDER)
{
 
  m_parent = parent;

  Connect(wxEVT_PAINT, wxPaintEventHandler(OdoPanel::OnPaint));
  Connect(wxEVT_SIZE, wxSizeEventHandler(OdoPanel::OnSize));

}

void OdoPanel::OnPaint(wxPaintEvent& event)
{
  wxFont font(9, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL,
            wxFONTWEIGHT_NORMAL, false, wxT("Courier 10 Pitch"));

  int border_width = 20;
  wxPaintDC dc(this);
  wxColour color;
  wxSize size = GetSize();
  int width = size.GetWidth();
  int height = size.GetHeight();

  wxPoint center(width/2, height/2);
  wxCoord radius = ((width<height)?(width/2 - border_width):(height/2 - border_width));
  wxCoord r = radius;

  color.Set(wxT("#7f8382"));
  dc.SetPen(wxPen(color, border_width, wxSOLID));
  dc.SetBrush(wxBrush(color));
  dc.DrawCircle(center, r);

  r = radius * 0.96;
  color.Set(wxT("#30363c"));
  dc.SetPen(wxPen(color, border_width, wxSOLID));
  dc.SetBrush(wxBrush(color));
  dc.DrawCircle(center, r);
//  dc.SetFont(font);

  color.Set(wxT("#7d8c8f"));
  r = radius * 0.8;
  dc.SetBrush(wxBrush(color));
  dc.DrawCircle(center, r);

  color.Set(wxT("#000906"));
  r = radius * 0.72;
  dc.SetBrush(wxBrush(color));
  dc.DrawCircle(center, r);

}

void OdoPanel::OnSize(wxSizeEvent& event)
{
  Refresh();
}

OdoMeter::OdoMeter(const wxString& title)
       : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(350, 200))
{

  wxPanel *panel = new wxPanel(this, wxID_ANY);
  wxPanel *centerPanel = new wxPanel(panel, wxID_ANY);

  m_slider = new wxSlider(centerPanel, ID_SLIDER, 75, 0, 750, wxPoint(-1, -1), 
      wxSize(150, -1), wxSL_LABELS);

  wxBoxSizer *vbox = new wxBoxSizer(wxVERTICAL);
  wxBoxSizer *hbox = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *hbox2 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *hbox3 = new wxBoxSizer(wxHORIZONTAL);

  m_wid = new OdoPanel(panel, wxID_ANY);
  hbox->Add(m_wid, 1, wxEXPAND);

  hbox2->Add(centerPanel, 1, wxEXPAND);
  hbox3->Add(m_slider, 0, wxTOP | wxLEFT, 35);

  centerPanel->SetSizer(hbox3);

  vbox->Add(hbox2, 1, wxEXPAND);
  vbox->Add(hbox, 0, wxEXPAND);

  panel->SetSizer(vbox);
  m_slider->SetFocus();

  Connect(ID_SLIDER, wxEVT_COMMAND_SLIDER_UPDATED, 
      wxScrollEventHandler(OdoMeter::OnScroll)); 

  Centre();

}

void OdoMeter::OnScroll(wxScrollEvent& WXUNUSED(event))
{
  m_wid->Refresh();
}

