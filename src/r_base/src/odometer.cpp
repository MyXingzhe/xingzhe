
#include <math.h>
#include <wx/wx.h>

#include "odometer.h"

float dial[] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
              10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,
              20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,
              30.0,31.0,32.0,33.0,34.0,35.0,36.0,37.0,38.0,39.0,
              40.0,41.0,42.0,43.0,44.0,45.0,46.0,47.0,48.0,49.0,
              50.0,51.0,52.0,53.0,54.0,55.0,56.0,57.0,58.0,59.0, 60.0};
//              60.0,61.0,62.0,63.0,64.0,65.0,66.0,67.0,68.0,69.0,
//              70.0,71.0,72.0,73.0,74.0,75.0,76.0,77.0,78.0,79.0,
//              80.0,81.0,82.0,83.0,84.0,85.0,86.0,87.0,88.0,89.0,
//              90.0,91.0,92.0,93.0,94.0,95.0,96.0,97.0,98.0,99.0,
//               };

int dialsize = sizeof(dial)/sizeof(dial[0]);

wxString grad_color[] = {wxT("#000010"), wxT("#000110"), wxT("#000210"), wxT("#000310"), wxT("#000410"), wxT("#000510"), wxT("#000610"), wxT("#000710") };

OdoPanel::OdoPanel(wxPanel *parent, int id)
      : wxPanel(parent, id, wxDefaultPosition, wxSize(-1, 30), wxSUNKEN_BORDER)
{
 
  m_parent = parent;

  Connect(wxEVT_PAINT, wxPaintEventHandler(OdoPanel::OnPaint));
  Connect(wxEVT_SIZE, wxSizeEventHandler(OdoPanel::OnSize));

}

void OdoPanel::OnPaint(wxPaintEvent& event)
{
  int border_width = 20;
  wxPaintDC dc(this);
  wxColour color;
  wxSize size = GetSize();
  int width = size.GetWidth();
  int height = size.GetHeight();
  int i;

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

  color.Set(wxT("#7d8c8f"));
  r = radius * 0.8;
  dc.SetBrush(wxBrush(color));
  dc.DrawCircle(center, r);

  for(i=0;i<8;i++)
  {
    color.Set(grad_color[i]);
    dc.SetPen(wxPen(color, border_width*0.3, wxSOLID));
    dc.SetBrush(wxBrush(color));
    dc.DrawCircle(center, r);
    r = r*0.99;
    dc.SetBrush(wxBrush(color));
    dc.DrawCircle(center, r);
    r *= 0.99;
  }

  color.Set(wxT("#000906"));
  r = radius * 0.72;
  dc.SetBrush(wxBrush(color));
  dc.DrawCircle(center, r);

  color.Set(wxT("#000c09"));
  dc.SetPen(wxPen(color, border_width*0.3, wxSOLID));
  dc.SetBrush(wxBrush(color));
  dc.DrawCircle(center, r);

  color.Set(wxT("#ffffff"));
  dc.SetPen(wxPen(color, border_width*0.1, wxSOLID));
  dc.SetBrush(wxBrush(color));
  wxFont font(radius/20, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_SLANT,
            wxFONTWEIGHT_NORMAL, false, wxT("Courier 10 Pitch"));

  dc.SetFont(font);
  dc.SetTextForeground(*wxWHITE);
  for(i=0;i<dialsize;i++)
  {
    if((i%5)==0)
    {
      dc.DrawLine(center.x - radius*(cos(dial[i]/60.0*M_PI)), center.y - radius*(sin(dial[i]/60.0*M_PI)),
                  center.x - radius*0.85*(cos(dial[i]/60.0*M_PI)), center.y - radius*0.85*(sin(dial[i]/60.0*M_PI)));

      dc.DrawRotatedText(wxString::Format("%d", (int)dial[i]), center.x - radius*0.85*(cos(dial[i]/60.0*M_PI)), center.y - radius*0.85*(sin(dial[i]/60.0*M_PI)), dial[i]/60.0*M_PI);
    }
    else
    {
      dc.DrawLine(center.x - radius*(cos(dial[i]/60.0*M_PI)), center.y - radius*(sin(dial[i]/60.0*M_PI)),
                  center.x - radius*0.9*(cos(dial[i]/60.0*M_PI)), center.y - radius*0.9*(sin(dial[i]/60.0*M_PI)));
    }
  }


  color.Set(wxT("#ff0030"));
  r = radius * 0.72;
  float speed = 18.0;
  dc.SetPen(wxPen(color, border_width*0.3, wxSOLID));
  dc.DrawLine(center.x - radius*(cos(speed/60.0*M_PI)), center.y - radius*(sin(speed/60.0*M_PI)),
              center.x - radius*0.35*(cos(speed/60.0*M_PI)), center.y - radius*0.35*(sin(speed/60.0*M_PI)));


}

void OdoPanel::OnSize(wxSizeEvent& event)
{
  Refresh();
}
