/*
 *  BelugaVideoSetupDialog.cpp
 *
 *  Created by Daniel Swain on 3/12/11
 *
 */

#include "BelugaVideoSetupDialog.h"

#include <wx/filepicker.h>
#include <wx/filefn.h>
#include <wx/image.h>

#include "CalibrationDataFile.h"

class imageCanvas : public wxWindow
{
protected:
    IplImage* m_pImage;
	wxBitmap m_Bitmap;
	int w, h;
public:
	imageCanvas(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size)
		: wxWindow(parent, id, pos, size),
          m_pImage(NULL),
          m_Bitmap(size.x, size.y),
          w(size.x),
          h(size.y)
	{
	};

	~imageCanvas()
	{
        if(m_pImage)
        {
            cvReleaseImage(&m_pImage);
        }
	};

	void OnPaint(wxPaintEvent& event)
	{
		wxPaintDC dc(this);
		PrepareDC(dc);

		dc.DrawBitmap(m_Bitmap, 0, 0);
	};

	void setImage(IplImage* image)
	{
        if(!m_pImage || (m_pImage->width != image->width) || (m_pImage->height != image->height))
        {
            if(m_pImage){cvReleaseImage(&m_pImage);}
            m_pImage = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 3);
        }
        cvConvertImage(image, m_pImage, CV_CVTIMG_SWAP_RB );
		wxImage tmp = wxImage(m_pImage->width, m_pImage->height,
                              (unsigned char*)m_pImage->imageData, true);
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
    ID_CALPICKER_0,
    ID_CALPICKER_1,
    ID_CALPICKER_2,
    ID_CALPICKER_3,
    ID_MASKPICKER_0,
	ID_MASKPICKER_1,
	ID_MASKPICKER_2,
	ID_MASKPICKER_3,
	ID_PANEL_0,
	ID_PANEL_1,
	ID_PANEL_2,
	ID_PANEL_3
};

Beluga_VideoSetupDialog::Beluga_VideoSetupDialog(MT_Capture* capture,
                                                 std::vector<std::string> names,
                                                 std::vector<std::string*> calibPaths,
                                                 std::vector<std::string*> maskPaths,
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
      m_vspCalibPaths(calibPaths),
      m_vspMaskPaths(maskPaths),
      m_bCamerasStarted(false),
      m_pIndexMap(indexMap)
{

    m_sEntryWD = wxGetCwd();
    
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

        m_pCalibrationPickerCtrls[i] = new wxFilePickerCtrl(this,
                                                     ID_CALPICKER_0+i,
                                                     wxEmptyString,
                                                     wxT("Select a Calibration File"),
                                                     wxT("*.*"),
                                                     wxDefaultPosition,
                                                     wxDefaultSize,
                                                     wxFLP_DEFAULT_STYLE | wxFLP_CHANGE_DIR);
        Connect(ID_CALPICKER_0+i,
                wxEVT_COMMAND_FILEPICKER_CHANGED,
                wxFileDirPickerEventHandler(Beluga_VideoSetupDialog::validateCalibration));
        
        m_pCalibrationPickerCtrls[i]->SetPath(MT_StringToWxString(*m_vspCalibPaths[i]));

        m_pMaskPickerCtrls[i] = new wxFilePickerCtrl(this,
                                                     ID_MASKPICKER_0+i,
                                                     wxEmptyString,
                                                     wxT("Select a Mask File"),
                                                     MT_FILTER_IMAGE_FILES,
                                                     wxDefaultPosition,
                                                     wxDefaultSize,
                                                     wxFLP_DEFAULT_STYLE | wxFLP_CHANGE_DIR);
        Connect(ID_MASKPICKER_0+i,
                wxEVT_COMMAND_FILEPICKER_CHANGED,
                wxFileDirPickerEventHandler(Beluga_VideoSetupDialog::validateMask));
        
        m_pMaskPickerCtrls[i]->SetPath(MT_StringToWxString(*m_vspMaskPaths[i]));
        
        
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
    vbox00->Add(new wxStaticText(this, -1, wxT("Parameter File")), 0, wxTOP, 10);
    vbox00->Add(m_pCalibrationPickerCtrls[1]);
    vbox00->Add(new wxStaticText(this, -1, wxT("Mask Image")), 0, wxTOP, 10);
    vbox00->Add(m_pMaskPickerCtrls[1]);
    
    vbox01->Add(new wxStaticText(this, -1, wxT("Camera I")));
    vbox01->Add(m_pChoices[0]);
    vbox01->Add(new wxStaticText(this, -1, wxT("Parameter File")), 0, wxTOP, 10);
    vbox01->Add(m_pCalibrationPickerCtrls[0]);
    vbox01->Add(new wxStaticText(this, -1, wxT("Mask Image")), 0, wxTOP, 10);
    vbox01->Add(m_pMaskPickerCtrls[0]);
    
    vbox10->Add(new wxStaticText(this, -1, wxT("Camera III")));
    vbox10->Add(m_pChoices[2]);
    vbox10->Add(new wxStaticText(this, -1, wxT("Parameter File")), 0, wxTOP, 10);
    vbox10->Add(m_pCalibrationPickerCtrls[2]);
    vbox10->Add(new wxStaticText(this, -1, wxT("Mask Image")), 0, wxTOP, 10);
	vbox10->Add(m_pMaskPickerCtrls[2]);
    
    vbox11->Add(new wxStaticText(this, -1, wxT("Camera IV")));
    vbox11->Add(m_pChoices[3]);
    vbox11->Add(new wxStaticText(this, -1, wxT("Parameter File")), 0, wxTOP, 10);
    vbox11->Add(m_pCalibrationPickerCtrls[3]);
    vbox11->Add(new wxStaticText(this, -1, wxT("Mask Image")), 0, wxTOP, 10);
	vbox11->Add(m_pMaskPickerCtrls[3]);
    

    vbox0->Add(grid0, 0, wxALL, 10);

    grid0->Add(vbox00);
	grid0->Add(m_pImageCanvases[1]);
	grid0->Add(m_pImageCanvases[0]);
    grid0->Add(vbox01);    
    grid0->Add(vbox10);
	grid0->Add(m_pImageCanvases[2]);
	grid0->Add(m_pImageCanvases[3]);
    grid0->Add(vbox11);

	wxButton* doneButton = new wxButton(this,
                            wxID_OK,
                            wxT("Done"));
	doneButton->SetDefault();
    vbox0->Add(doneButton,
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

void Beluga_VideoSetupDialog::validateCalibration(wxFileDirPickerEvent& event)
{
    wxString path = event.GetPath();
    int i = event.GetId() - ID_CALPICKER_0;
    Beluga_CalibrationDataFile calFile((const char*) path.mb_str());
    if(!calFile.didLoadOK())
    {
        MT_ShowErrorDialog(this,
                           wxT("Could not validate calibration file ")
                           + path);
        event.SetPath(MT_StringToWxString(*m_vspCalibPaths[i]));
    }
}

void Beluga_VideoSetupDialog::validateMask(wxFileDirPickerEvent& event)
{
    wxString path = event.GetPath();
    int i = event.GetId() - ID_MASKPICKER_0;

    IplImage* m = cvLoadImage((const char*) path.mb_str(), CV_LOAD_IMAGE_UNCHANGED);
    if(!m || m->nChannels != 1 || m->width != 640 || m->height != 480)
    {
        MT_ShowErrorDialog(this,
                           wxT("Mask image must be a 640x480 grayscale image"));
        event.SetPath(MT_StringToWxString(*m_vspMaskPaths[i]));
    }
}

void Beluga_VideoSetupDialog::onDoneClicked(wxCommandEvent& event)
{
    for(unsigned int i = 0; i < 4; i++)
    {
        *m_vspCalibPaths[i] = std::string(m_pCalibrationPickerCtrls[i]->GetPath().mb_str());
        *m_vspMaskPaths[i] = std::string(m_pMaskPickerCtrls[i]->GetPath().mb_str());

		Beluga_CalibrationDataFile f(m_vspCalibPaths[i]->c_str());
		if(!f.didLoadOK())
		{
			MT_ShowErrorDialog(this, wxT("All calibration data files must be specified."));
			return;
		}
    }

    wxSetWorkingDirectory(m_sEntryWD);
    
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
    int i = MT_MAX(m_pCapture->getNumInterfacesOpen()-1, 0);
    if(i >= 3)
    {
        return;
    }
    
    for(; i < 4; i++)
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
