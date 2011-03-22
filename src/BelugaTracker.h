#ifndef BELUGATRACKER_H
#define BELUGATRACKER_H

#include "MT_Core.h"
#include "MT_Tracking.h"

#include "BelugaRobot.h"

const double DEFAULT_SIGMA_POSITION = 4.0; /* pixels */
const double DEFAULT_SIGMA_HEADING = 0.26; /* rad ~= 15 deg */
const double DEFAULT_SIGMA_SPEED = 1.0; /* pixels/frame */
const double DEFAULT_SIGMA_POSITION_MEAS = 1.0; /* pixels */
const double DEFAULT_SIGMA_HEADING_MEAS = 0.087; /* rad ~= 5 deg */

typedef std::vector<double> t_p_history;

class BelugaTracker : public MT_TrackerBase
{
private:
    /* frames */
	IplImage* m_pGSFrames[4];
	IplImage* m_pHSVFrames[4];
	IplImage* m_pHFrames[4];
	IplImage* m_pSFrames[4];
	IplImage* m_pVFrames[4];
	IplImage* m_pThreshFrames[4];

	void HSVSplit(IplImage* frame, int i);

    /* blobber parameters */
    unsigned int m_iBlobValThresh;
	unsigned int m_iBlobAreaThreshLow;
	unsigned int m_iBlobAreaThreshHigh;
    
    CvRect m_SearchArea[4];
	unsigned int m_iSearchAreaPadding;

    /* only used to add to XDF */
    int m_iStartFrame;
    int m_iStopFrame;

    MT_GSThresholder* m_pGSThresholder[4];
    GYBlobber* m_pGYBlobber[4];

    std::vector<MT_UKF_struct*> m_vpUKF;
    CvMat* m_pQ;
    CvMat* m_pR;
    CvMat* m_px0;
    CvMat* m_pz;
    double m_dSigmaPosition;
    double m_dSigmaHeading;
    double m_dSigmaSpeed;
    double m_dSigmaPositionMeas;
    double m_dSigmaHeadingMeas;
    
    double m_dPrevSigmaPosition;
    double m_dPrevSigmaHeading;
    double m_dPrevSigmaSpeed;
    double m_dPrevSigmaPositionMeas;
    double m_dPrevSigmaHeadingMeas;

    bool m_bShowBlobs;
    bool m_bShowTracking;

    /* operational variables */
    double m_dDt;
    int m_iFrameCounter;
    int m_iNObj;

    std::vector<double> m_vdBlobs_X;
    std::vector<double> m_vdBlobs_Y; 
    std::vector<double> m_vdBlobs_Area; 
    std::vector<double> m_vdBlobs_Orientation;
    std::vector<double> m_vdBlobs_MajorAxis;
    std::vector<double> m_vdBlobs_MinorAxis;
    std::vector<double> m_vdBlobs_Speed;

    std::vector<double> m_vdTracked_X;
    std::vector<double> m_vdTracked_Y;
    std::vector<double> m_vdTracked_Heading;
    std::vector<double> m_vdTracked_Speed;

    std::vector<t_p_history> m_vdHistories_X;
    std::vector<t_p_history> m_vdHistories_Y;

	unsigned int m_iFrameWidth;
    unsigned int m_iFrameHeight;
    
public:
    /* constructor */
    BelugaTracker(IplImage* ProtoFrame, unsigned int n_obj);
    /* destructor - note we need the virtual destructor */
    virtual ~BelugaTracker();
    
    /* Initialization */
    void doInit(IplImage* ProtoFrame);

    /* Memory allocation / deallocation */
    void createFrames();
    void releaseFrames();

    void setDiffThresh(int thresh){m_iBlobValThresh = thresh;};
    void setStartStopFrames(int start_frame, int stop_frame)
    {m_iStartFrame = start_frame; m_iStopFrame = stop_frame;};

    void doTrain(IplImage* frame);

    void initDataFile();
    void writeData();

    /* Main tracking functions */
    void doTracking(IplImage* frame);
	void doTracking(IplImage* frames[4]);

    void doGLDrawing(int flags);

	std::vector<double> getBelugaState(unsigned int i);
	double getBelugaX(unsigned int i){if(i >= m_vdTracked_X.size()){return 0;} else {return m_vdTracked_X[i];}};
	double getBelugaY(unsigned int i){if(i >= m_vdTracked_Y.size()){return 0;} else {return m_iFrameHeight - m_vdTracked_Y[i];}};

};

#endif /* BELUGATRACKER_H */
