#ifndef BELUGA_VIDEOSETUPDIALOG_H
#define BELUGA_VIDEOSETUPDIALOG_H

#include "MT_Tracking.h"

class wxFilePickerCtrl;
class wxFileDirPickerEvent;
class imageCanvas;

class Beluga_VideoSetupDialog : public wxDialog
{
protected:
    std::vector<std::string> m_vsNames;
    std::vector<std::string*> m_vspCalibPaths;
    MT_Capture* m_pCapture;

    bool m_bCamerasStarted;

    unsigned int* m_pIndexMap;

    wxChoice* m_pChoices[4];
    wxFilePickerCtrl* m_pFilePickerCtrls[4];
	imageCanvas* m_pImageCanvases[4];

    wxString m_sEntryWD;

    void onDoneClicked(wxCommandEvent& event);

    void onCameraSelected(wxCommandEvent& event);
    void SwapCameras(int quad1, int quad2);

    void validateCalibration(wxFileDirPickerEvent& event);

public:
    Beluga_VideoSetupDialog(MT_Capture* capture,
                            std::vector<std::string> names,
                            std::vector<std::string*> calibPaths,
                            unsigned int* indexMap,
                            wxWindow* parent,
                            wxWindowID id = wxID_ANY);

    void StartCameras();
    void UpdateView();
    
};

#endif // BELUGA_VIDEOSETUPDIALOG_H
