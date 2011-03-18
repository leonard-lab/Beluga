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
    std::vector<std::string*> m_vspMaskPaths;    
    MT_Capture* m_pCapture;

    bool m_bCamerasStarted;

    unsigned int* m_pIndexMap;

    wxChoice* m_pChoices[4];
    wxFilePickerCtrl* m_pCalibrationPickerCtrls[4];
    wxFilePickerCtrl* m_pMaskPickerCtrls[4];    
	imageCanvas* m_pImageCanvases[4];

    wxString m_sEntryWD;

    void onDoneClicked(wxCommandEvent& event);

    void onCameraSelected(wxCommandEvent& event);
    void SwapCameras(int quad1, int quad2);

    void validateCalibration(wxFileDirPickerEvent& event);
    void validateMask(wxFileDirPickerEvent& event);    

public:
    Beluga_VideoSetupDialog(MT_Capture* capture,
                            std::vector<std::string> names,
                            std::vector<std::string*> calibPaths,
                            std::vector<std::string*> maskPaths,
                            unsigned int* indexMap,
                            wxWindow* parent,
                            wxWindowID id = wxID_ANY);

    void StartCameras();
    void UpdateView();
    
};

#endif // BELUGA_VIDEOSETUPDIALOG_H
