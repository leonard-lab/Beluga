#ifndef BELUGATRACKER_H
#define BELUGATRACKER_H

#include "MT_Core.h"
#include "MT_Tracking.h"

#include "BelugaRobot.h"

#include "CoordinateTransforms.h"

const double DEFAULT_SIGMA_POSITION = 4.0; 
const double DEFAULT_SIGMA_Z = 1.0;
const double DEFAULT_SIGMA_ZDOT = 1.0;
const double DEFAULT_SIGMA_SPEED = 0.1;
const double DEFAULT_SIGMA_HEADING = 0.26; 
const double DEFAULT_SIGMA_OMEGA = 2.6; 
const double DEFAULT_SIGMA_POSITION_MEAS = 1.0;
const double DEFAULT_SIGMA_Z_MEAS = 1.0; 
const double DEFAULT_SIGMA_HEADING_MEAS = 0.087;


/* we'll derive from the MT_DSGYA_Segmenter so that we can
 * intercept the "usePrevious" method and use it to flag when
 * an object is not found in a particular camera */
class BelugaSegmenter : public MT_DSGYA_Segmenter
{
public:
    BelugaSegmenter(class BelugaTracker* tracker)
        : m_pTracker(tracker), MT_DSGYA_Segmenter() {};
    ~BelugaSegmenter(){};

    void usePrevious(MT_DSGYA_Blob* obj, unsigned int i);
	bool areAdjacent(MT_DSGYA_Blob* obj, const YABlob& blob);
private:
    class BelugaTracker* m_pTracker;
};

typedef std::vector<double> t_p_history;

class BelugaTracker : public MT_TrackerBase
{
private:
    /* frames */
	IplImage* m_pGSFrames[4];
	IplImage* m_pUndistortedFrames[4];
	IplImage* m_pHSVFrames[4];
	IplImage* m_pHFrames[4];
	IplImage* m_pSFrames[4];
	IplImage* m_pVFrames[4];
	IplImage* m_pHThreshFrames[4];
	IplImage* m_pSThreshFrames[4];
	IplImage* m_pVThreshFrames[4];
	IplImage* m_pThreshFrames[4];

	IplImage* m_pMasks[4];

	IplImage* m_pTempFrame1;

	CvMat* m_pCameraMatrices[4];
	CvMat* m_pDistortionCoeffs[4];
	CvMat* m_pRotationVectors[4];
	CvMat* m_pRotationMatrices[4];
	CvMat* m_pTranslationVectors[4];

	IplImage* m_pUndistortMapX;
	IplImage* m_pUndistortMapY;

    double m_dWaterDepth;
    CTWithWater m_CoordinateTransforms[4];

	void HSVSplit(IplImage* frame, int i);

    /* blobber parameters */
    unsigned int m_iBlobValThresh;
	unsigned int m_iBlobAreaThreshLow;
	unsigned int m_iBlobAreaThreshHigh;
    double m_dOverlapFactor;

	unsigned int m_iVThresh;
	unsigned int m_iHThresh_Low;
	unsigned int m_iHThresh_High;
	unsigned int m_iSThresh_Low;
	unsigned int m_iSThresh_High;

	YABlobber m_YABlobber;

	std::vector<MT_DSGYA_Blob> m_vBlobs[4];
	std::vector<MT_DSGYA_Blob> m_vPredictedBlobs[4];
	std::vector<YABlob> m_vInitBlobs[4];    
    unsigned int m_iAssignmentRows[4];
    unsigned int m_iAssignmentCols[4];
    std::vector<unsigned int> m_viAssignments[4];
    bool m_bNoHistory;

	std::vector<CvRect> m_SearchArea[4];
	std::vector<std::vector<unsigned int> > m_SearchIndexes[4];
	unsigned int m_iSearchAreaPadding;

    /* only used to add to XDF */
    int m_iStartFrame;
    int m_iStopFrame;

    std::vector<MT_UKF_struct*> m_vpUKF;
    CvMat* m_pQ;
    CvMat* m_pR;
    CvMat* m_px0;
    CvMat* m_pz;
    CvMat* m_pu;
    double m_dSigmaPosition;
    double m_dSigmaZ;
    double m_dSigmaZDot;
    double m_dSigmaSpeed;    
    double m_dSigmaHeading;
    double m_dSigmaOmega;
    
    double m_dSigmaPositionMeas;
    double m_dSigmaHeadingMeas;    
    double m_dSigmaZMeas;
    
    double m_dPrevSigmaPosition;
    double m_dPrevSigmaZ;
    double m_dPrevSigmaZDot;
    double m_dPrevSigmaHeading;
    double m_dPrevSigmaSpeed;
    double m_dPrevSigmaOmega;
    
    double m_dPrevSigmaPositionMeas;
    double m_dPrevSigmaHeadingMeas;
    double m_dPrevSigmaZMeas;    

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
	std::vector<double> m_vdaTracked_XC[4];
	std::vector<double> m_vdaTracked_YC[4];
	std::vector<double> m_vdTracked_Z;
    std::vector<double> m_vdTracked_ZDot;
    std::vector<double> m_vdTracked_Speed;    
    std::vector<double> m_vdTracked_Heading;
    std::vector<double> m_vdTracked_Omega;

	std::vector<std::vector<double> > m_vvdMeas_X;
	std::vector<std::vector<double> > m_vvdMeas_Y;
	std::vector<std::vector<double> > m_vvdMeas_Hdg;
	std::vector<std::vector<unsigned int> > m_vviMeas_A;
	std::vector<std::vector<unsigned int> > m_vviMeas_Cam;

    double m_dCurrentTime;
	std::vector<double> m_vdDepthMeasurement;
	std::vector<double> m_vdSpeedCommand;
	std::vector<double> m_vdVerticalCommand;
	std::vector<double> m_vdTurnCommand;

	std::vector<bool> m_vbLastMeasValid;
	std::vector<t_p_history> m_vdHistories_X;
    std::vector<t_p_history> m_vdHistories_Y;

	unsigned int m_iFrameWidth;
    unsigned int m_iFrameHeight;

    MT_TrackerFrameGroup* m_pAuxFrameGroups[3];

    unsigned int m_iCurrentCamera;
    std::vector<bool> m_vbValidMeasCamObj[4];

    double m_dInitAdjacencyThresh;
    unsigned int testInitAdjacency(double x1, double y1, double x2, double y2);

    void generatePredictedBlobs(unsigned int cam_number);    
	void calculateTrackedPositionsInCameras();
	void calculatePositionOfObjectInCamera(unsigned int obj_num, unsigned int cam_num, double* u, double* v);

    ////////////////////////////////////////////////////////////
    /* functions that get called during each tracking step */
    
    /* calculates a time stamp */
    void doTimeKeeping();

    /* try to initialize the state vectors with whatever measurements
     * we found, returns true on success */
    bool tryInitStateVectors();
    
    /* updates UKF parameters/matrices if they have changed */
    void updateUKFParameters();

    /* image processing step */
    void doImageProcessingInCamera(unsigned int cam_number, const IplImage* frame);

    /* first finding step - i.e., find image connected components */
    void doBlobFindingInCamera(unsigned int cam_number);

    /* extract measurements for each object and set up the UKF
     * for updating */
    void getObjectMeasurements(unsigned int obj_number);

    /* update the state for each object
     *   usually using the UKF but handling special cases */
    void updateObjectState(unsigned int obj_number);
    
    /* apply UKF to an object */
    void applyUKFToObject(unsigned int obj_number);

    /* handles the first frame */
    void useAverageMeasurementForObject(unsigned int obj_number);

    /* handles when no measurement is available */
    void usePredictedStateForObject(unsigned int obj_number);

    ////////////////////////////////////////////////////////////

protected:
    friend class BelugaSegmenter;
    void notifyNoMeasurement(unsigned int obj_num);
    
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

	void setMasks(const char* maskfile1,
		const char* maskfile2,
		const char* maskfile3,
		const char* maskfile4);
	void setCalibrations(const char* calibfile1,
		const char* calibfile2,
		const char* calibfile3,
		const char* calibfile4);

    MT_TrackerFrameGroup* getAuxFrameGroup(int i){return m_pAuxFrameGroups[i];};

    /* Main tracking functions */
    void doTracking(IplImage* frame);
	void doTracking(IplImage* frames[4]);

    void doGLDrawing(int flags);

	std::vector<double> getBelugaState(unsigned int i);
	double getBelugaX(unsigned int i){if(i >= m_vdTracked_X.size()){return 0;} else {return m_vdTracked_X[i];}};
	double getBelugaY(unsigned int i){if(i >= m_vdTracked_Y.size()){return 0;} else {return m_vdTracked_Y[i];}};
	double getBelugaZ(unsigned int i){if(i >= m_vdTracked_Z.size()){return 0;} else {return m_vdTracked_Z[i];}};

	void getWorldXYZFromImageXYAndDepthInCamera(double* x,
		double* y,
		double* z,
		double u,
		double v,
		double d,
		bool undistort,
		unsigned int camera);
	void getCameraXYFromWorldXYandDepth(int* camera, double* u, double* v, double x, double y, double depth, bool distort);

	void setRobotData(const std::vector<double>& depth_meas,
		const std::vector<double>& speed,
		const std::vector<double>& vert,
		const std::vector<double>& turn);
};

#endif /* BELUGATRACKER_H */
