/*
 *  BelugaVideoSetupDialog.cpp
 *
 *  Created by Daniel Swain on 3/12/11
 *
 */

#include "BelugaVideoSetupDialog.h"

#include <wx/filepicker.h>
#include <wx/image.h>

class imageCanvas : public wxWindow
{
protected:
	wxBitmap m_Bitmap;
	int w, h;
public:
	imageCanvas(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size)
		: wxWindow(parent, id, pos, size),
		m_Bitmap(size.x, size.y),
		w(size.x),
		h(size.y)
	{
	};

	~imageCanvas()
	{
	};

	void OnPaint(wxPaintEvent& event)
	{
		wxPaintDC dc(this);
		PrepareDC(dc);

		dc.DrawBitmap(m_Bitmap, 0, 0);
	};

	void setImage(IplImage* image)
	{
		wxImage tmp = wxImage(image->width, image->height, (unsigned char*)image->imageData, true);
		m_Bitmap = wxBitmap(tmp.Scale(w,h));
		this->Refresh();
	}

	DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(imageCanvas, wxWindow)
EVT_PAINT(imageCanvas::OnPaint)
END_EVENT_TABLE()

static const wxSize g_canvasSize = wxSize(320, 240);

enum
{
    ID_LISTBOX_0 = wxID_HIGHEST + 1,
    ID_LISTBOX_1,
    ID_LISTBOX_2,
    ID_LISTBOX_3,
    ID_FILEPICKER_0,
    ID_FILEPICKER_1,
    ID_FILEPICKER_2,
    ID_FILEPICKER_3,    
	ID_PANEL_0,
	ID_PANEL_1,
	ID_PANEL_2,
	ID_PANEL_3
};

Beluga_VideoSetupDialog::Beluga_VideoSetupDialog(MT_Capture* capture,
                                                 std::vector<std::string> names,
                                                 unsigned int* indexMap,
                                                 wxWindow* parent,
                                                 wxWindowID id)
    : wxDialog(parent,
               id,
               wxT("Video source configuration"),
               wxDefaultPosition,
               wxDefaultSize,
               wxDEFAULT_DIALOG_STYLE),
      m_pCapture(capture),
      m_vsNames(names),
      m_bCamerasStarted(false),
      m_pIndexMap(indexMap)
{
    wxArrayString choices = wxArrayString();
    choices.empty();
    for(unsigned int i = 0; i < 4; i++)
    {
        choices.Add(wxString(names[i]));
    }
    
    for(unsigned int i = 0; i < 4; i++)
    {
        m_pChoices[i] = new wxChoice(this,
                                        ID_LISTBOX_0+i,
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        choices);
        m_pChoices[i]->SetSelection(m_pIndexMap[i]);
        Connect(ID_LISTBOX_0+i,
                wxEVT_COMMAND_CHOICE_SELECTED,
                wxCommandEventHandler(Beluga_VideoSetupDialog::onCameraSelected));

        m_pFilePickerCtrls[i] = new wxFilePickerCtrl(this,
                                                     ID_FILEPICKER_0+i);

		m_pImageCanvases[i] = new imageCanvas(this,
			ID_PANEL_0+i,
			wxDefaultPosition,
			g_canvasSize);
                                                     
    }

    wxBoxSizer* vbox0 = new wxBoxSizer(wxVERTICAL);
    wxFlexGridSizer* grid0 = new wxFlexGridSizer(2, 4, 5, 15);

    wxBoxSizer* vbox00 = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* vbox01 = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* vbox10 = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* vbox11 = new wxBoxSizer(wxVERTICAL);    
        
    vbox00->Add(new wxStaticText(this, -1, wxT("Camera II")));
    vbox00->Add(m_pChoices[1]);
    vbox00->Add(new wxStaticText(this, -1, wxT("Parameter File")));
    vbox00->Add(m_pFilePickerCtrls[1]);
    
    vbox01->Add(new wxStaticText(this, -1, wxT("Camera I")));
    vbox01->Add(m_pChoices[0]);
    vbox01->Add(new wxStaticText(this, -1, wxT("Parameter File")));
    vbox01->Add(m_pFilePickerCtrls[0]);
    
    vbox10->Add(new wxStaticText(this, -1, wxT("Camera III")));
    vbox10->Add(m_pChoices[2]);
    vbox10->Add(new wxStaticText(this, -1, wxT("Parameter File")));
    vbox10->Add(m_pFilePickerCtrls[2]);
    
    vbox11->Add(new wxStaticText(this, -1, wxT("Camera IV")));
    vbox11->Add(m_pChoices[3]);
    vbox11->Add(new wxStaticText(this, -1, wxT("Parameter File")));
    vbox11->Add(m_pFilePickerCtrls[3]);

    vbox0->Add(grid0, 0, wxALL, 10);

    grid0->Add(vbox00);
	grid0->Add(m_pImageCanvases[1]);
	grid0->Add(m_pImageCanvases[0]);
    grid0->Add(vbox01);    
    grid0->Add(vbox10);
	grid0->Add(m_pImageCanvases[2]);
	grid0->Add(m_pImageCanvases[3]);
    grid0->Add(vbox11);

    vbox0->Add(new wxButton(this,
                            wxID_OK,
                            wxT("Done")),
               0,
               wxALIGN_RIGHT | wxRIGHT,
               10);

    Connect(wxID_OK,
            wxEVT_COMMAND_BUTTON_CLICKED,
            wxCommandEventHandler(Beluga_VideoSetupDialog::onDoneClicked));

    SetSizerAndFit(vbox0);
                            
}

void Beluga_VideoSetupDialog::onCameraSelected(wxCommandEvent& event)
{
    int quad_of_selecting = event.GetId() - ID_LISTBOX_0;
    int quad_of_selected;
    
    for(unsigned int i = 0; i < 4; i++)
    {
        if(m_pIndexMap[i] == event.GetSelection())
        {
            quad_of_selected = i;
        }
    }
    
    SwapCameras(quad_of_selecting, quad_of_selected);
}

void Beluga_VideoSetupDialog::onDoneClicked(wxCommandEvent& event)
{
    EndModal(wxID_OK);
}

void Beluga_VideoSetupDialog::SwapCameras(int quad1, int quad2)
{
    int cam1 = m_pIndexMap[quad1];
    int cam2 = m_pIndexMap[quad2];

    m_pIndexMap[quad1] = cam2;
    m_pIndexMap[quad2] = cam1;

    m_pChoices[quad1]->SetSelection(cam2);
    m_pChoices[quad2]->SetSelection(cam1);

    UpdateView();
}

void Beluga_VideoSetupDialog::StartCameras()
{
    m_bCamerasStarted = true;
    for(unsigned int i = 0; i < 4; i++)
    {
        m_pCapture->initCaptureFromCameraNumber(i);
    }
}

void Beluga_VideoSetupDialog::UpdateView()
{
    if(!m_bCamerasStarted)
    {
        StartCameras();
    }
    
    for(unsigned int i = 0; i < 4; i++)
    {
        IplImage* f = m_pCapture->getFrame(MT_FC_NEXT_FRAME, m_pIndexMap[i]);
        f = m_pCapture->getFrame(MT_FC_NEXT_FRAME, m_pIndexMap[i]);   

		m_pImageCanvases[i]->setImage(f);

    }
}
