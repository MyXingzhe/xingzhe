/////////////////////////////////////////////////////////////////////////////
// Name:        toolbar.cpp
// Purpose:     wxToolBar sample
// Author:      Julian Smart
// Modified by:
// Created:     04/01/98
// Copyright:   (c) Julian Smart
// Licence:     wxWindows licence
/////////////////////////////////////////////////////////////////////////////

// ============================================================================
// declarations
// ============================================================================

// ----------------------------------------------------------------------------
// headers
// ----------------------------------------------------------------------------

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif

#include "wx/toolbar.h"
#include "wx/log.h"
#include "wx/image.h"
#include "wx/filedlg.h"
#include "wx/colordlg.h"
#include "wx/srchctrl.h"
#include "wx/wrapsizer.h"

#include "odometer.h"
#include "switch.h"


// define this to use XPMs everywhere (by default, BMPs are used under Win)
// BMPs use less space, but aren't compiled into the executable on other platforms
#ifdef __WINDOWS__
    #define USE_XPM_BITMAPS 0
#else
    #define USE_XPM_BITMAPS 1
#endif

// Define this as 0 for the platforms not supporting controls in toolbars
#define USE_CONTROLS_IN_TOOLBAR 1

// ----------------------------------------------------------------------------
// resources
// ----------------------------------------------------------------------------

#include "base.xpm"

#if USE_XPM_BITMAPS
    #include "new.xpm"
    #include "connect.xpm"
    #include "disconnect.xpm"
    #include "open.xpm"
    #include "save.xpm"
    #include "copy.xpm"
    #include "cut.xpm"
    #include "preview.xpm"  // paste XPM
    #include "print.xpm"
    #include "exit.xpm"
#endif // USE_XPM_BITMAPS

enum Positions
{
    TOOLBAR_LEFT,
    TOOLBAR_TOP,
    TOOLBAR_RIGHT,
    TOOLBAR_BOTTOM
};


// ----------------------------------------------------------------------------
// classes
// ----------------------------------------------------------------------------

// Define a new application
class BaseApp : public wxApp
{
public:
    bool OnInit();
};

// Define a new frame
class RBaseFrame: public wxFrame
{
public:
    RBaseFrame(wxFrame *parent,
            wxWindowID id = wxID_ANY,
            const wxString& title = wxT("wxToolBar Sample"),
            const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize,
            long style = wxDEFAULT_FRAME_STYLE|wxCLIP_CHILDREN|wxNO_FULL_REPAINT_ON_RESIZE|wxMAXIMIZE);
    virtual ~RBaseFrame();

    void PopulateToolbar(wxToolBarBase* toolBar);
    void RecreateToolbar();

    void OnQuit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);

    void OnSize(wxSizeEvent& event);

    void OnToggleToolbar(wxCommandEvent& event);
    void CreateLeftToolbar();
    void OnToggleHorizontalText(wxCommandEvent& WXUNUSED(event));


    void OnToggleToolbarSize(wxCommandEvent& event);
    void OnChangeOrientation(wxCommandEvent& event);
    void OnToggleToolbarRows(wxCommandEvent& event);
    void OnToggleTooltips(wxCommandEvent& event);
    void OnToggleCustomDisabled(wxCommandEvent& event);

    void OnEnablePrint(wxCommandEvent& WXUNUSED(event)) { DoEnablePrint(); }
    void OnDeletePrint(wxCommandEvent& WXUNUSED(event)) { DoDeletePrint(); }
    void OnInsertPrint(wxCommandEvent& event);
    void OnChangeToolTip(wxCommandEvent& event);
    void OnToggleSearch(wxCommandEvent& event);

    void OnToolbarStyle(wxCommandEvent& event);
    void OnToolbarBgCol(wxCommandEvent& event);
    void OnToolbarCustomBitmap(wxCommandEvent& event);
    void OnToolbarConnect(wxCommandEvent& event);

    void OnToolLeftClick(wxCommandEvent& event);

    void OnUpdateCopyAndCut(wxUpdateUIEvent& event);
    void OnUpdateToggleHorzText(wxUpdateUIEvent& event);
    void OnUpdateToggleRadioBtn(wxUpdateUIEvent& event)
        { event.Enable( m_tbar != NULL ); }

    void OnLeftScroll(wxScrollEvent& event);
    void OnRightScroll(wxScrollEvent& event);
private:
    void DoEnablePrint();
    void DoDeletePrint();

    void LayoutChildren();

    bool                m_smallToolbar,
                        m_horzText,
                        m_useCustomDisabled,
                        m_showTooltips;
    size_t              m_rows;             // 1 or 2 only

    // the number of print buttons we have (they're added/removed dynamically)
    size_t              m_nPrint;

    bool                m_connected;

    // store toolbar position for future use
    Positions           m_toolbarPosition;

    wxPanel             *m_ctrl_panel;

    wxPanel             *m_panel;

    wxToolBar           *m_tbar;
    OdoPanel            *m_odo;;
    wxSlider            *m_leftslider;
    wxSlider            *m_rightslider;

    // the path to the custom bitmap for the test toolbar tool
    wxString            m_pathBmp;
    SwitchPanel         *m_leftswitch;
    SwitchPanel         *m_rightswitch;

    // the search tool, initially NULL
    wxToolBarToolBase *m_searchTool;

    wxDECLARE_EVENT_TABLE();
};

// ----------------------------------------------------------------------------
// constants
// ----------------------------------------------------------------------------

const int ID_TOOLBAR = 500;

static const long TOOLBAR_STYLE = wxTB_FLAT | wxTB_DOCKABLE | wxTB_TEXT;

enum
{
    // toolbar menu items
    IDM_TOOLBAR_TOGGLE_TOOLBAR = 200,
    IDM_TOOLBAR_TOGGLE_HORIZONTAL_TEXT,
    IDM_TOOLBAR_TOGGLETOOLBARSIZE,
    IDM_TOOLBAR_TOGGLETOOLBARROWS,
    IDM_TOOLBAR_TOGGLETOOLTIPS,
    IDM_TOOLBAR_TOGGLECUSTOMDISABLED,
    IDM_TOOLBAR_SHOW_TEXT,
    IDM_TOOLBAR_SHOW_ICONS,
    IDM_TOOLBAR_SHOW_BOTH,
    IDM_TOOLBAR_BG_COL,
    IDM_TOOLBAR_CUSTOM_PATH,
    IDM_TOOLBAR_TOP_ORIENTATION,
    IDM_TOOLBAR_LEFT_ORIENTATION,
    IDM_TOOLBAR_BOTTOM_ORIENTATION,
    IDM_TOOLBAR_RIGHT_ORIENTATION,

    // tools menu items
    IDM_TOOLBAR_CONNECT,
    IDM_TOOLBAR_DISCONNECT,

    IDM_TOOLBAR_ENABLEPRINT,
    IDM_TOOLBAR_DELETEPRINT,
    IDM_TOOLBAR_INSERTPRINT,
    IDM_TOOLBAR_TOGGLESEARCH,
    IDM_TOOLBAR_CHANGE_TOOLTIP,

    IDM_LEFTWHEEL_SLIDER,
    IDM_RIGHTWHEEL_SLIDER,

    ID_COMBO = 1000
};

// ----------------------------------------------------------------------------
// event tables
// ----------------------------------------------------------------------------

// Notice that wxID_HELP will be processed for the 'About' menu and the toolbar
// help button.

wxBEGIN_EVENT_TABLE(RBaseFrame, wxFrame)
    EVT_SIZE(RBaseFrame::OnSize)

    EVT_MENU(wxID_EXIT, RBaseFrame::OnQuit)

    EVT_MENU(IDM_TOOLBAR_TOGGLE_TOOLBAR, RBaseFrame::OnToggleToolbar)
    EVT_MENU(IDM_TOOLBAR_TOGGLE_HORIZONTAL_TEXT, RBaseFrame::OnToggleHorizontalText)

    EVT_MENU_RANGE(IDM_TOOLBAR_TOP_ORIENTATION, IDM_TOOLBAR_RIGHT_ORIENTATION, RBaseFrame::OnChangeOrientation)
    EVT_MENU(IDM_TOOLBAR_TOGGLETOOLBARSIZE, RBaseFrame::OnToggleToolbarSize)
    EVT_MENU(IDM_TOOLBAR_TOGGLETOOLBARROWS, RBaseFrame::OnToggleToolbarRows)
    EVT_MENU(IDM_TOOLBAR_TOGGLETOOLTIPS, RBaseFrame::OnToggleTooltips)
    EVT_MENU(IDM_TOOLBAR_TOGGLECUSTOMDISABLED, RBaseFrame::OnToggleCustomDisabled)

    EVT_MENU(IDM_TOOLBAR_ENABLEPRINT, RBaseFrame::OnEnablePrint)
    EVT_MENU(IDM_TOOLBAR_DELETEPRINT, RBaseFrame::OnDeletePrint)
    EVT_MENU(IDM_TOOLBAR_INSERTPRINT, RBaseFrame::OnInsertPrint)
    EVT_MENU(IDM_TOOLBAR_TOGGLESEARCH, RBaseFrame::OnToggleSearch)
    EVT_MENU(IDM_TOOLBAR_CHANGE_TOOLTIP, RBaseFrame::OnChangeToolTip)

    EVT_MENU_RANGE(IDM_TOOLBAR_SHOW_TEXT, IDM_TOOLBAR_SHOW_BOTH,
                   RBaseFrame::OnToolbarStyle)
    EVT_MENU(IDM_TOOLBAR_BG_COL, RBaseFrame::OnToolbarBgCol)

    EVT_MENU(IDM_TOOLBAR_CUSTOM_PATH, RBaseFrame::OnToolbarCustomBitmap)

    EVT_MENU(IDM_TOOLBAR_CONNECT, RBaseFrame::OnToolbarConnect)
    EVT_MENU(IDM_TOOLBAR_DISCONNECT, RBaseFrame::OnToolbarConnect)

    EVT_UPDATE_UI(wxID_COPY, RBaseFrame::OnUpdateCopyAndCut)
    EVT_UPDATE_UI(wxID_CUT, RBaseFrame::OnUpdateCopyAndCut)

    EVT_UPDATE_UI(IDM_TOOLBAR_TOGGLE_HORIZONTAL_TEXT,
                  RBaseFrame::OnUpdateToggleHorzText)
wxEND_EVENT_TABLE()

// ============================================================================
// implementation
// ============================================================================

// ----------------------------------------------------------------------------
// BaseApp
// ----------------------------------------------------------------------------

IMPLEMENT_APP(BaseApp)

// The `main program' equivalent, creating the windows and returning the
// main frame
bool BaseApp::OnInit()
{
    if ( !wxApp::OnInit() )
        return false;

    // Create the main frame window
    RBaseFrame* frame = new RBaseFrame((wxFrame *) NULL, wxID_ANY,
                                 wxT("wxToolBar Sample"),
                                  wxPoint(0, 0), wxSize(1280, 800));

    frame->Show(true);

#if wxUSE_STATUSBAR
    frame->SetStatusText(wxT("Hello, wxWidgets"));
#endif

    wxInitAllImageHandlers();

    return true;
}


void RBaseFrame::RecreateToolbar()
{
    // delete and recreate the toolbar
    wxToolBarBase *toolBar = GetToolBar();
    long style = toolBar ? toolBar->GetWindowStyle() : TOOLBAR_STYLE;

    if (toolBar && m_searchTool && m_searchTool->GetToolBar() == NULL)
    {
        // see ~RBaseFrame()
        toolBar->AddTool(m_searchTool);
    }
    m_searchTool = NULL;

    delete toolBar;

    SetToolBar(NULL);

    style &= ~(wxTB_HORIZONTAL | wxTB_VERTICAL | wxTB_BOTTOM | wxTB_RIGHT | wxTB_HORZ_LAYOUT);
    switch( m_toolbarPosition )
    {
        case TOOLBAR_LEFT:
            style |= wxTB_LEFT;
            break;
        case TOOLBAR_TOP:
            style |= wxTB_TOP;
            break;
        case TOOLBAR_RIGHT:
            style |= wxTB_RIGHT;
            break;
        case TOOLBAR_BOTTOM:
        style |= wxTB_BOTTOM;
        break;
    }

    if ( m_showTooltips )
        style &= ~wxTB_NO_TOOLTIPS;
    else
        style |= wxTB_NO_TOOLTIPS;

    if ( style & wxTB_TEXT && !(style & wxTB_NOICONS) && m_horzText )
        style |= wxTB_HORZ_LAYOUT;

    toolBar = CreateToolBar(style, ID_TOOLBAR);

    PopulateToolbar(toolBar);
}

void RBaseFrame::PopulateToolbar(wxToolBarBase* toolBar)
{
    // Set up toolbar
    enum
    {
        Tool_connect,
        Tool_disconnect,
        Tool_open,
        Tool_save,
        Tool_copy,
        Tool_cut,
        Tool_paste,
        Tool_print,
        Tool_exit,
        Tool_Max
    };

    wxBitmap toolBarBitmaps[Tool_Max];

#if USE_XPM_BITMAPS
    #define INIT_TOOL_BMP(bmp) \
        toolBarBitmaps[Tool_##bmp] = wxBitmap(bmp##_xpm)
#else // !USE_XPM_BITMAPS
    #define INIT_TOOL_BMP(bmp) \
        toolBarBitmaps[Tool_##bmp] = wxBITMAP(bmp)
#endif // USE_XPM_BITMAPS/!USE_XPM_BITMAPS

    INIT_TOOL_BMP(connect);
    INIT_TOOL_BMP(disconnect);
    INIT_TOOL_BMP(open);
    INIT_TOOL_BMP(save);
    INIT_TOOL_BMP(copy);
    INIT_TOOL_BMP(cut);
    INIT_TOOL_BMP(paste);
    INIT_TOOL_BMP(print);
    INIT_TOOL_BMP(exit);

    int w = toolBarBitmaps[Tool_connect].GetWidth(),
        h = toolBarBitmaps[Tool_connect].GetHeight();

    if ( !m_smallToolbar )
    {
        w *= 2;
        h *= 2;

        for ( size_t n = Tool_connect; n < WXSIZEOF(toolBarBitmaps); n++ )
        {
            toolBarBitmaps[n] =
                wxBitmap(toolBarBitmaps[n].ConvertToImage().Scale(w, h));
        }
    }

    // this call is actually unnecessary as the toolbar will adjust its tools
    // size to fit the biggest icon used anyhow but it doesn't hurt neither
    toolBar->SetToolBitmapSize(wxSize(w, h));

    toolBar->AddTool(wxID_NEW, wxT("Conect"),
                     toolBarBitmaps[Tool_connect], wxNullBitmap, wxITEM_NORMAL,
                     wxT("New file"), wxT("This is help for new file tool"));


    toolBar->AddTool(wxID_OPEN, wxT("Disconnect"),
                     toolBarBitmaps[Tool_disconnect], wxNullBitmap, wxITEM_NORMAL,
                     wxT("Open file"), wxT("This is help for open file tool"));

#if USE_CONTROLS_IN_TOOLBAR
    // adding a combo to a vertical toolbar is not very smart
    if ( !toolBar->IsVertical() )
    {
        wxComboBox *combo = new wxComboBox(toolBar, ID_COMBO, wxEmptyString, wxDefaultPosition, wxSize(100,-1) );
        combo->Append(wxT("This"));
        combo->Append(wxT("is a"));
        combo->Append(wxT("combobox"));
        combo->Append(wxT("in a"));
        combo->Append(wxT("toolbar"));
        toolBar->AddControl(combo, wxT("Combo Label"));
    }
#endif // USE_CONTROLS_IN_TOOLBAR

    toolBar->AddTool(wxID_SAVE, wxT("Save"), toolBarBitmaps[Tool_save], wxT("Toggle button 1"), wxITEM_CHECK);

    toolBar->AddSeparator();
    toolBar->AddTool(wxID_COPY, wxT("Copy"), toolBarBitmaps[Tool_copy], wxT("Toggle button 2"), wxITEM_CHECK);
    toolBar->AddTool(wxID_CUT, wxT("Cut"), toolBarBitmaps[Tool_cut], wxT("Toggle/Untoggle help button"));
    toolBar->AddTool(wxID_PASTE, wxT("Paste"), toolBarBitmaps[Tool_paste], wxT("Paste"));
    toolBar->AddSeparator();

    if ( m_useCustomDisabled )
    {
        wxBitmap bmpDisabled(w, h);
        {
            wxMemoryDC dc;
            dc.SelectObject(bmpDisabled);
            dc.DrawBitmap(toolBarBitmaps[Tool_print], 0, 0);

            wxPen pen(*wxRED, 5);
            dc.SetPen(pen);
            dc.DrawLine(0, 0, w, h);
        }

        toolBar->AddTool(wxID_PRINT, wxT("Print"), toolBarBitmaps[Tool_print],
                         bmpDisabled);
    }
    else
    {
        toolBar->AddTool(wxID_PRINT, wxT("Print"), toolBarBitmaps[Tool_print],
                         wxT("Delete this tool. This is a very long tooltip to test whether it does the right thing when the tooltip is more than Windows can cope with."));
    }

    // add a stretchable space before the "Help" button to make it
    // right-aligned
    toolBar->AddStretchableSpace();
    toolBar->AddTool(wxID_EXIT, wxT("Exit"), toolBarBitmaps[Tool_exit], wxT("Exit button"), wxITEM_CHECK);

    if ( !m_pathBmp.empty() )
    {
        // create a tool with a custom bitmap for testing
        wxImage img(m_pathBmp);
        if ( img.IsOk() )
        {
            if ( img.GetWidth() > w && img.GetHeight() > h )
                img = img.GetSubImage(wxRect(0, 0, w, h));

            toolBar->AddSeparator();
            toolBar->AddTool(wxID_ANY, wxT("Custom"), img);
        }
    }

    // after adding the buttons to the toolbar, must call Realize() to reflect
    // the changes
    toolBar->Realize();

    toolBar->SetRows(toolBar->IsVertical() ? toolBar->GetToolsCount() / m_rows
                                           : m_rows);
}

// ----------------------------------------------------------------------------
// RBaseFrame
// ----------------------------------------------------------------------------

// Define my frame constructor
RBaseFrame::RBaseFrame(wxFrame* parent,
                 wxWindowID id,
                 const wxString& title,
                 const wxPoint& pos,
                 const wxSize& size,
                 long style)
       : wxFrame(parent, id, title, pos, size, style)
{
    m_tbar = NULL;

    m_smallToolbar = false;
    m_horzText = false;
    m_useCustomDisabled = false;
    m_showTooltips = true;
    m_searchTool = NULL;
    m_connected = false;

    m_rows = 1;
    m_nPrint = 1;

#if wxUSE_STATUSBAR
    // Give it a status line
    CreateStatusBar();
#endif

    // Give it an icon
    SetIcon(wxICON(base));

    // Make a menubar
    wxMenu *tbarMenu = new wxMenu;
    tbarMenu->AppendCheckItem(IDM_TOOLBAR_TOGGLE_TOOLBAR,
                              wxT("Toggle &toolbar\tCtrl-Z"),
                              wxT("Show or hide the toolbar"));

    tbarMenu->AppendCheckItem(IDM_TOOLBAR_TOGGLE_HORIZONTAL_TEXT,
                              wxT("Toggle hori&zontal text\tCtrl-H"),
                              wxT("Show text under/alongside the icon"));

    tbarMenu->AppendCheckItem(IDM_TOOLBAR_TOGGLETOOLBARSIZE,
                              wxT("&Toggle toolbar size\tCtrl-S"),
                              wxT("Toggle between big/small toolbar"));

    tbarMenu->AppendCheckItem(IDM_TOOLBAR_TOGGLETOOLBARROWS,
                              wxT("Toggle number of &rows\tCtrl-R"),
                              wxT("Toggle number of toolbar rows between 1 and 2"));

    tbarMenu->AppendCheckItem(IDM_TOOLBAR_TOGGLETOOLTIPS,
                              wxT("Show &tooltips\tCtrl-L"),
                              wxT("Show tooltips for the toolbar tools"));

    tbarMenu->AppendCheckItem(IDM_TOOLBAR_TOGGLECUSTOMDISABLED,
                              wxT("Use c&ustom disabled images\tCtrl-U"),
                              wxT("Switch between using system-generated and custom disabled images"));


    tbarMenu->AppendSeparator();
    tbarMenu->AppendRadioItem(IDM_TOOLBAR_TOP_ORIENTATION,
                              wxT("Set toolbar at the top of the window"),
                              wxT("Set toolbar at the top of the window"));
    tbarMenu->AppendRadioItem(IDM_TOOLBAR_LEFT_ORIENTATION,
                              wxT("Set toolbar at the left of the window"),
                              wxT("Set toolbar at the left of the window"));
    tbarMenu->AppendRadioItem(IDM_TOOLBAR_BOTTOM_ORIENTATION,
                              wxT("Set toolbar at the bottom of the window"),
                              wxT("Set toolbar at the bottom of the window"));
    tbarMenu->AppendRadioItem(IDM_TOOLBAR_RIGHT_ORIENTATION,
                              wxT("Set toolbar at the right edge of the window"),
                              wxT("Set toolbar at the right edge of the window"));
    tbarMenu->AppendSeparator();

    tbarMenu->AppendRadioItem(IDM_TOOLBAR_SHOW_TEXT, wxT("Show &text\tCtrl-Alt-T"));
    tbarMenu->AppendRadioItem(IDM_TOOLBAR_SHOW_ICONS, wxT("Show &icons\tCtrl-Alt-I"));
    tbarMenu->AppendRadioItem(IDM_TOOLBAR_SHOW_BOTH, wxT("Show &both\tCtrl-Alt-B"));
    tbarMenu->AppendSeparator();
    tbarMenu->Append(IDM_TOOLBAR_BG_COL, wxT("Choose bac&kground colour..."));
    tbarMenu->Append(IDM_TOOLBAR_CUSTOM_PATH, wxT("Custom &bitmap...\tCtrl-B"));

    wxMenu *toolMenu = new wxMenu;
    toolMenu->Append(IDM_TOOLBAR_ENABLEPRINT, wxT("&Enable print button\tCtrl-E"));
    toolMenu->Append(IDM_TOOLBAR_DELETEPRINT, wxT("&Delete print button\tCtrl-D"));
    toolMenu->Append(IDM_TOOLBAR_INSERTPRINT, wxT("&Insert print button\tCtrl-I"));
    toolMenu->AppendCheckItem(IDM_TOOLBAR_TOGGLESEARCH, wxT("Toggle &search field\tCtrl-F"));
    toolMenu->AppendSeparator();
    toolMenu->Append(IDM_TOOLBAR_CHANGE_TOOLTIP, wxT("Change tooltip of \"New\""));

    wxMenu *fileMenu = new wxMenu;
    fileMenu->Append(wxID_EXIT, wxT("E&xit\tAlt-X"), wxT("Quit toolbar sample") );

    wxMenuBar* menuBar = new wxMenuBar( wxMB_DOCKABLE );

    menuBar->Append(fileMenu, wxT("&File"));
    menuBar->Append(tbarMenu, wxT("&Toolbar"));
    menuBar->Append(toolMenu, wxT("Tool&s"));

    // Associate the menu bar with the frame
    SetMenuBar(menuBar);

    menuBar->Check(IDM_TOOLBAR_TOGGLE_TOOLBAR, true);
    menuBar->Check(IDM_TOOLBAR_SHOW_BOTH, true);
    menuBar->Check(IDM_TOOLBAR_TOGGLETOOLTIPS, true);

    menuBar->Check(IDM_TOOLBAR_TOP_ORIENTATION, true );
    m_toolbarPosition = TOOLBAR_TOP;

//    CreateLeftToolbar();

    // Create the toolbar
    RecreateToolbar();

    m_panel = new wxPanel(this);

    // Root sizer, vertical
    wxSizer * const sizerRoot = new wxBoxSizer(wxHORIZONTAL);

    // Some toolbars in a wrap sizer
    wxSizer * const sizerTop = new wxWrapSizer( wxHORIZONTAL );
    sizerRoot->Add(sizerTop, wxSizerFlags().Expand().Border());

    // A number of checkboxes inside a wrap sizer
    wxSizer *sizerMid = new wxStaticBoxSizer(wxVERTICAL, m_panel, "");

    wxBoxSizer *hbox = new wxBoxSizer(wxHORIZONTAL);
    m_odo = new OdoPanel(m_panel, wxID_ANY);
    hbox->Add(m_odo, 1, wxEXPAND);

    wxFlexGridSizer *m_sizer1 = new wxFlexGridSizer(2, 3, 5, 5);
    wxPanel *m_panel1 = new wxPanel(m_panel);
    m_panel1->SetSizer(m_sizer1);
    m_leftswitch = new SwitchPanel(m_panel1, wxID_ANY);
    m_leftslider = new wxSlider(m_panel1, wxID_ANY,
                            0, 0, 100,
                            wxDefaultPosition, wxSize(400,20),
                            wxSL_LABELS);

    m_rightswitch = new SwitchPanel(m_panel1, wxID_ANY);
    m_rightslider = new wxSlider(m_panel1, wxID_ANY,
                            0, 0, 100,
                            wxDefaultPosition, wxSize(400,20),
                            wxSL_LABELS);

    Connect(IDM_LEFTWHEEL_SLIDER, wxEVT_COMMAND_SLIDER_UPDATED, 
            wxScrollEventHandler(RBaseFrame::OnLeftScroll)); 
    Connect(IDM_RIGHTWHEEL_SLIDER, wxEVT_COMMAND_SLIDER_UPDATED, 
            wxScrollEventHandler(RBaseFrame::OnRightScroll)); 

    m_sizer1->Add(new wxStaticText(m_panel1, wxID_ANY, wxT("LEFT WHEEL"), wxDefaultPosition, wxSize(100,10), 0, wxT("LEFT WHEEL")));
    m_sizer1->Add(m_leftswitch, 1, wxEXPAND);
    m_sizer1->Add(m_leftslider, 1, wxEXPAND);
    m_sizer1->Add(new wxStaticText(m_panel1, wxID_ANY, wxT("LEFT WHEEL"), wxDefaultPosition, wxSize(100,10), 0, wxT("RIGHT WHEEL")));
    m_sizer1->Add(m_rightswitch, 1, wxEXPAND);
    m_sizer1->Add(m_rightslider, 1, wxEXPAND);
    m_sizer1->AddGrowableCol(1, 1);
    m_sizer1->AddGrowableRow(0, 1);
    m_sizer1->AddGrowableRow(1, 1);

    sizerMid->Add(hbox, wxSizerFlags(100).Expand());
    sizerMid->Add(m_panel1, wxSizerFlags(100).Expand());
    sizerRoot->Add(sizerMid, wxSizerFlags(100).Expand().Border());

    // A shaped item inside a box sizer
    wxSizer *sizerBottom = new wxStaticBoxSizer(wxVERTICAL, m_panel,
                                                "With wxSHAPED item");
    wxSizer *sizerBottomBox = new wxBoxSizer(wxHORIZONTAL);
    sizerBottom->Add(sizerBottomBox, wxSizerFlags(100).Expand());

    sizerBottomBox->Add(new wxListBox(m_panel, wxID_ANY,
                                        wxPoint(0, 0), wxSize(70, 70)),
                        wxSizerFlags().Expand().Shaped());
    sizerBottomBox->AddSpacer(10);
    sizerBottomBox->Add(new wxCheckBox(m_panel, wxID_ANY,
                                        "A much longer option..."),
                        wxSizerFlags(100).Border());
    sizerRoot->Add(sizerBottom, wxSizerFlags(100).Expand().Border());

    // OK Button
    sizerRoot->Add(new wxButton(m_panel, wxID_OK),
                    wxSizerFlags().Centre().DoubleBorder());

    // Set sizer for the panel
    m_panel->SetSizer(sizerRoot);

    Show();
}

RBaseFrame::~RBaseFrame()
{
    if ( m_searchTool && !m_searchTool->GetToolBar() )
    {
        // we currently can't delete a toolbar tool ourselves, so we have to
        // attach it to the toolbar just for it to be deleted, this is pretty
        // ugly and will need to be changed
        GetToolBar()->AddTool(m_searchTool);
    }
}

void RBaseFrame::LayoutChildren()
{
    wxSize size = GetClientSize();

    int offset;
    if ( m_tbar )
    {
        m_tbar->SetSize(0, 0, wxDefaultCoord, size.y);

        offset = m_tbar->GetSize().x;
    }
    else
    {
        offset = 0;
    }

    m_panel->SetSize(offset, 0, size.x - offset, size.y);
}

void RBaseFrame::OnSize(wxSizeEvent& event)
{
    if ( m_tbar )
    {
        LayoutChildren();
    }
    else
    {
        event.Skip();
    }
}

void RBaseFrame::OnToggleToolbar(wxCommandEvent& WXUNUSED(event))
{
    wxToolBar *tbar = GetToolBar();

    if ( !tbar )
    {
        RecreateToolbar();
    }
    else
    {
        // notice that there is no need to call SetToolBar(NULL) here (although
        // this it is harmless to do and it must be called if you do not delete
        // the toolbar but keep it for later reuse), just delete the toolbar
        // directly and it will reset the associated frame toolbar pointer
        delete tbar;
    }
}

void RBaseFrame::OnToggleHorizontalText(wxCommandEvent& WXUNUSED(event))
{
    m_horzText = !m_horzText;

    RecreateToolbar();
}

void RBaseFrame::CreateLeftToolbar()
{
    long style = GetToolBar() ? GetToolBar()->GetWindowStyle()
                              : TOOLBAR_STYLE;
    style &= ~wxTB_HORIZONTAL;
    style |= wxTB_VERTICAL;

    m_tbar = new wxToolBar(this, wxID_ANY,
                           wxDefaultPosition, wxDefaultSize,
                           style);

    m_tbar->SetMargins(4, 4);

    m_tbar->AddTool(IDM_TOOLBAR_CONNECT, wxT("Connect"), wxBITMAP(connect));
    m_tbar->AddTool(IDM_TOOLBAR_DISCONNECT, wxT("Disconnect"), wxBITMAP(disconnect));
    if(m_connected)
    {
        m_tbar->EnableTool(IDM_TOOLBAR_CONNECT, false);
        m_tbar->EnableTool(IDM_TOOLBAR_DISCONNECT, true);
    }
    else
    {
        m_tbar->EnableTool(IDM_TOOLBAR_CONNECT, true);
        m_tbar->EnableTool(IDM_TOOLBAR_DISCONNECT, false);
    }
    m_tbar->AddSeparator();
    m_tbar->AddTool(wxID_EXIT, wxT("Exit"), wxBITMAP(exit));

    m_tbar->Realize();

//    LayoutChildren();
}

void RBaseFrame::OnToggleToolbarSize(wxCommandEvent& WXUNUSED(event))
{
    m_smallToolbar = !m_smallToolbar;

    RecreateToolbar();
}

void RBaseFrame::OnToggleToolbarRows(wxCommandEvent& WXUNUSED(event))
{
    // m_rows may be only 1 or 2
    m_rows = 3 - m_rows;

    wxToolBar* const toolBar = GetToolBar();
    toolBar->SetRows(toolBar->IsVertical() ? toolBar->GetToolsCount() / m_rows
                                           : m_rows);

    //RecreateToolbar(); -- this is unneeded
}

void RBaseFrame::OnToggleTooltips(wxCommandEvent& WXUNUSED(event))
{
    m_showTooltips = !m_showTooltips;

    RecreateToolbar();
}

void RBaseFrame::OnToggleCustomDisabled(wxCommandEvent& WXUNUSED(event))
{
    m_useCustomDisabled = !m_useCustomDisabled;

    RecreateToolbar();
}

void RBaseFrame::OnChangeOrientation(wxCommandEvent& event)
{
    switch( event.GetId() )
    {
        case IDM_TOOLBAR_LEFT_ORIENTATION:
            m_toolbarPosition = TOOLBAR_LEFT;
            break;
        case IDM_TOOLBAR_TOP_ORIENTATION:
            m_toolbarPosition = TOOLBAR_TOP;
            break;
        case IDM_TOOLBAR_RIGHT_ORIENTATION:
            m_toolbarPosition = TOOLBAR_RIGHT;
            break;
        case IDM_TOOLBAR_BOTTOM_ORIENTATION:
            m_toolbarPosition = TOOLBAR_BOTTOM;
            break;
    }
    RecreateToolbar();
}

void RBaseFrame::OnQuit(wxCommandEvent& WXUNUSED(event))
{
    Close(true);
}

void RBaseFrame::OnAbout(wxCommandEvent& event)
{

    (void)wxMessageBox(wxT("wxWidgets toolbar sample"), wxT("About wxToolBar"));
}

void RBaseFrame::OnToolLeftClick(wxCommandEvent& event)
{
    if (event.GetId() == wxID_COPY)
    {
        DoEnablePrint();
    }

    if (event.GetId() == wxID_CUT)
    {
    }

    if (event.GetId() == wxID_PRINT)
    {
        DoDeletePrint();
    }
}

void RBaseFrame::OnToolbarConnect(wxCommandEvent& event)
{
    m_connected = !m_connected;
    if(m_connected)
    {
        m_tbar->EnableTool(IDM_TOOLBAR_CONNECT, false);
        m_tbar->EnableTool(IDM_TOOLBAR_DISCONNECT, true);
    }
    else
    {
        m_tbar->EnableTool(IDM_TOOLBAR_CONNECT, true);
        m_tbar->EnableTool(IDM_TOOLBAR_DISCONNECT, false);
    }
}

void RBaseFrame::DoEnablePrint()
{
    if ( !m_nPrint )
        return;

    wxToolBarBase *tb = GetToolBar();
    tb->EnableTool(wxID_PRINT, !tb->GetToolEnabled(wxID_PRINT));
}

void RBaseFrame::DoDeletePrint()
{
    if ( !m_nPrint )
        return;

    wxToolBarBase *tb = GetToolBar();
    tb->DeleteTool( wxID_PRINT );

    m_nPrint--;
}

void RBaseFrame::OnToggleSearch(wxCommandEvent& WXUNUSED(event))
{
    wxToolBarBase * const tb = GetToolBar();
    if ( !m_searchTool )
    {
        wxSearchCtrl * const srch = new wxSearchCtrl(tb, wxID_ANY, "needle");
        srch->SetMinSize(wxSize(80, -1));
        m_searchTool = tb->AddControl(srch);
    }
    else // tool already exists
    {
        wxControl * const win = m_searchTool->GetControl();
        if ( m_searchTool->GetToolBar() )
        {
            // attached now, remove it
            win->Hide();
            tb->RemoveTool(m_searchTool->GetId());
        }
        else // tool exists in detached state, attach it back
        {
            tb->AddTool(m_searchTool);
            win->Show();
        }
    }

    tb->Realize();
}

void RBaseFrame::OnUpdateCopyAndCut(wxUpdateUIEvent& event)
{
}

void RBaseFrame::OnUpdateToggleHorzText(wxUpdateUIEvent& event)
{
    wxToolBar *tbar = GetToolBar();
    event.Enable( tbar &&
                    tbar->HasFlag(wxTB_TEXT) &&
                        !tbar->HasFlag(wxTB_NOICONS) );
}

void RBaseFrame::OnChangeToolTip(wxCommandEvent& WXUNUSED(event))
{
    GetToolBar()->SetToolShortHelp(wxID_NEW, wxT("New toolbar button"));
}

void RBaseFrame::OnToolbarStyle(wxCommandEvent& event)
{
    long style = GetToolBar()->GetWindowStyle();
    style &= ~(wxTB_NOICONS | wxTB_HORZ_TEXT);

    switch ( event.GetId() )
    {
        case IDM_TOOLBAR_SHOW_TEXT:
            style |= wxTB_NOICONS | (m_horzText ? wxTB_HORZ_TEXT : wxTB_TEXT);
            break;

        case IDM_TOOLBAR_SHOW_ICONS:
            // nothing to do
            break;

        case IDM_TOOLBAR_SHOW_BOTH:
            style |= (m_horzText ? wxTB_HORZ_TEXT : wxTB_TEXT);
    }

    GetToolBar()->SetWindowStyle(style);
}

void RBaseFrame::OnToolbarBgCol(wxCommandEvent& WXUNUSED(event))
{
    wxColour col = wxGetColourFromUser
                   (
                    this,
                    GetToolBar()->GetBackgroundColour(),
                    "Toolbar background colour"
                   );
    if ( col.IsOk() )
    {
        GetToolBar()->SetBackgroundColour(col);
        GetToolBar()->Refresh();
    }
}

void RBaseFrame::OnToolbarCustomBitmap(wxCommandEvent& WXUNUSED(event))
{
    m_pathBmp = wxLoadFileSelector("custom bitmap", "");

    RecreateToolbar();
}

void RBaseFrame::OnInsertPrint(wxCommandEvent& WXUNUSED(event))
{
    m_nPrint++;

    wxToolBarBase *tb = GetToolBar();
    tb->InsertTool(0, wxID_PRINT, wxT("New print"),
                   wxBITMAP(print), wxNullBitmap,
                   wxITEM_NORMAL,
                   wxT("Delete this tool"),
                   wxT("This button was inserted into the toolbar"));

    // must call Realize() after adding a new button
    tb->Realize();
}

void RBaseFrame::OnLeftScroll(wxScrollEvent& WXUNUSED(event))
{
printf("on left scroll!\n");
    if(m_leftslider->GetValue()<60)
    {
        printf("on left scroll!\n");
        m_odo->SetSpeed(m_leftslider->GetValue());
    }
    
    Refresh();
}

void RBaseFrame::OnRightScroll(wxScrollEvent& WXUNUSED(event))
{
printf("on right scroll!\n");

    Refresh();
}
