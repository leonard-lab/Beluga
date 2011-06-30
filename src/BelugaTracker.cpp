#include "BelugaTracker.h"

#include "CalibrationDataFile.h"

#include "TrackerExtras.h"
#include "BelugaDynamics.h"

/* default parameter values */
const unsigned int DEFAULT_BG_THRESH = 60;
const double DEFAULT_MIN_BLOB_PERIMETER = 10;   
const double DEFAULT_MIN_BLOB_AREA = 10;        
const double DEFAULT_MAX_BLOB_PERIMETER = 1000; 
const double DEFAULT_MAX_BLOB_AREA = 1000;
const unsigned int DEFAULT_SEARCH_AREA_PADDING = 100;

const unsigned int DEFAULT_V_THRESH = 40;
const unsigned int DEFAULT_H_THRESH_HIGH = 130;
const unsigned int DEFAULT_H_THRESH_LOW = 10;
const unsigned int DEFAULT_S_THRESH_LOW = 0;
const unsigned int DEFAULT_S_THRESH_HIGH = 255;

const double DEFAULT_WATER_DEPTH = 2.286;

/* default color to draw blobs */
const MT_Color DEFAULT_BLOB_COLOR = MT_Red;

/* number of past positions to use when estimating orientation */
const unsigned int N_hist = 5;

const double DEFAULT_GATE_DIST2 = 100.0*100.0;

//#define VFLIP m_iFrameHeight -
#define VFLIP

/**********************************************************************
 * Tracker Class
 *********************************************************************/

/* Constructor - this gets called when we create a new instance of
 * this class.  It should initialize all of the member variables and
 * do whatever memory allocation is necessary. */
BelugaTracker::BelugaTracker(IplImage* ProtoFrame, unsigned int n_obj)
    : MT_TrackerBase(ProtoFrame),
      m_dWaterDepth(DEFAULT_WATER_DEPTH),
      m_iBlobValThresh(DEFAULT_BG_THRESH),
	  m_iBlobAreaThreshLow(DEFAULT_MIN_BLOB_AREA),
	  m_iBlobAreaThreshHigh(DEFAULT_MAX_BLOB_AREA),
	  m_iVThresh(DEFAULT_V_THRESH),
	  m_iHThresh_Low(DEFAULT_H_THRESH_LOW),
	  m_iHThresh_High(DEFAULT_H_THRESH_HIGH),
	  m_iSThresh_Low(DEFAULT_S_THRESH_LOW),
	  m_iSThresh_High(DEFAULT_S_THRESH_HIGH),
	  m_iSearchAreaPadding(DEFAULT_SEARCH_AREA_PADDING),
      m_iStartFrame(-1),
      m_iStopFrame(-1),
//      m_pGSThresholder(            NULL                        ),
 //     m_pGYBlobber(NULL),
      m_vpUKF(n_obj, NULL),
      m_dSigmaPosition(DEFAULT_SIGMA_POSITION),
      m_dSigmaHeading(DEFAULT_SIGMA_HEADING),
      m_dSigmaSpeed(DEFAULT_SIGMA_SPEED),
      m_dSigmaPositionMeas(DEFAULT_SIGMA_POSITION_MEAS),
      m_dSigmaHeadingMeas(DEFAULT_SIGMA_HEADING_MEAS),
      m_dPrevSigmaPosition(0),
      m_dPrevSigmaHeading(0),
      m_dPrevSigmaSpeed(0),
      m_dPrevSigmaPositionMeas(0),
      m_dPrevSigmaHeadingMeas(0),
      m_bShowBlobs(true),
      m_bShowTracking(true),
      m_dDt(0),
      m_iFrameCounter(0),
      m_iNObj(n_obj),
	  m_dCurrentTime(0),
      m_iFrameHeight(0),
	  m_iFrameWidth(0)
{
    doInit(ProtoFrame);
}

/* Destructor - basically the opposite of the destructor, gets called
 * whenever an object is deleted or goes out of scope.  It should
 * de-allocate any memory that was allocated */
BelugaTracker::~BelugaTracker()
{
	/*
    if(m_pGSThresholder[0])
    {
		for(int i = 0; i < 4; i++)
		{
			delete m_pGSThresholder[i];
		}
    }

    if(m_pGYBlobber[0])
    {
		for(int i = 0; i < 4; i++)
		{
			delete m_pGYBlobber[i];
		}
    }*/

    /* cvReleaseMat deallocates matrices given to it */
    cvReleaseMat(&m_px0);
    cvReleaseMat(&m_pz);
    cvReleaseMat(&m_pQ);
    cvReleaseMat(&m_pR);

	if(m_pHSVFrames[0])
	{
		for(int i = 0; i < 4; i++)
		{
			//cvReleaseImage(&m_pGSFrames[i]);
			cvReleaseImage(&m_pUndistortedFrames[i]);
			cvReleaseImage(&m_pHSVFrames[i]);
			cvReleaseImage(&m_pHFrames[i]);
			cvReleaseImage(&m_pSFrames[i]);
			cvReleaseImage(&m_pVFrames[i]);
			cvReleaseImage(&m_pThreshFrames[i]);
		}
		cvReleaseImage(&m_pTempFrame1);
    	cvReleaseImage(&m_pTempFrame2);
	}

	if(m_pMasks[0])
	{
		cvReleaseImage(&m_pMasks[0]);
		cvReleaseImage(&m_pMasks[1]);
		cvReleaseImage(&m_pMasks[2]);
		cvReleaseImage(&m_pMasks[3]);
	}

	if(m_pCameraMatrices[0])
	{
		for(int i = 0; i < 4; i++)
		{
			cvReleaseMat(&m_pCameraMatrices[i]);
			cvReleaseMat(&m_pDistortionCoeffs[i]);
			cvReleaseMat(&m_pRotationVectors[i]);
			cvReleaseMat(&m_pRotationMatrices[i]);
			cvReleaseMat(&m_pTranslationVectors[i]);
		}
	}

	if(m_pUndistortMapX)
	{
		cvReleaseImage(&m_pUndistortMapX);
		cvReleaseImage(&m_pUndistortMapY);
	}

    for(int i = 0; i < 3; i++)
    {
        delete m_pAuxFrameGroups[i];
    }

    /* MT_UKFFree frees up memory used by the UKF */
    for(int i = 0; i < m_iNObj; i++)
    {
        MT_UKFFree(&m_vpUKF[i]);
    }
}

/* this is the main initialization function and gets called by the
 * constructor.  It does memory allocation, etc. */
void BelugaTracker::doInit(IplImage* ProtoFrame)
{

	for(int i = 0; i < 4; i++)
	{
		m_SearchArea[i].resize(0);
		m_SearchIndexes[i].resize(0);
	}

    /* Not using the built-in tracked objects functions - setting
     * this pointer to N ULL will ensure that the appropriate code is
     * disabled  */
    m_pTrackedObjects = NULL;

    /* It's always a good idea to initialize pointers to NULL so that
       other pieces of code can use e.g. if(p) to check for allocation */
	for(int i = 0; i < 4; i++)
	{
		m_pGSFrames[i] = NULL;
		m_pThreshFrames[i] = NULL;

		m_pHSVFrames[i] = NULL;
		m_pHFrames[i] = NULL;
		m_pSFrames[i] = NULL;
		m_pVFrames[i] = NULL;

		m_pMasks[i] = NULL;

		m_pTempFrame1 = NULL;
		m_pTempFrame2 = NULL;

		m_pCameraMatrices[i] = NULL;
		m_pDistortionCoeffs[i] = NULL;
		m_pRotationVectors[i] = NULL;
		m_pRotationMatrices[i] = NULL;
		m_pTranslationVectors[i] = NULL;

		m_vBlobs[i].resize(0);
	}

	m_pUndistortMapX = NULL;
	m_pUndistortMapY = NULL;

    /* grab the frame height */
    m_iFrameHeight = ProtoFrame->height;
	m_iFrameWidth = ProtoFrame->width;

    /* Call the base class's doInit method.
     * This initializes variables to safe values and
     * calls doTrain to set the background frame,
     * frame sizes, and calls createFrames(). */
    MT_TrackerBase::doInit(ProtoFrame);

    /* resize all of our vectors. note that the std::vector object
       deallocates memory on its own, so we won't have to do that later */
    m_vdBlobs_X.resize(m_iNObj);
    m_vdBlobs_Y.resize(m_iNObj);          
    m_vdBlobs_Area.resize(m_iNObj);
    m_vdBlobs_Orientation.resize(m_iNObj);
    m_vdBlobs_MajorAxis.resize(m_iNObj);  
    m_vdBlobs_MinorAxis.resize(m_iNObj);  
    m_vdBlobs_Speed.resize(m_iNObj);
    m_viMatchAssignments.resize(m_iNObj);
    m_vdTracked_X.resize(m_iNObj);
    m_vdTracked_Y.resize(m_iNObj);
    m_vdTracked_Z.resize(m_iNObj);
	m_vvdMeas_X.resize(m_iNObj);
	m_vvdMeas_Y.resize(m_iNObj);
	m_vviMeas_A.resize(m_iNObj);
	m_vviMeas_Cam.resize(m_iNObj);
	m_vvdMeas_Hdg.resize(m_iNObj);
	for(int i = 0; i < 4; i++)
	{
		m_vdaTracked_XC[i].resize(m_iNObj);
		m_vdaTracked_YC[i].resize(m_iNObj);
	}
	for(int i = 0; i < m_iNObj; i++)
	{
		m_vvdMeas_X[i].resize(0);
		m_vvdMeas_Y[i].resize(0);
		m_vviMeas_A[i].resize(0);
		m_vviMeas_Cam[i].resize(0);
		m_vvdMeas_Hdg[i].resize(0);
	}
    m_vdTracked_Heading.resize(m_iNObj);
    m_vdTracked_Speed.resize(m_iNObj);

	m_vdDepthMeasurement.resize(0);
	m_vdSpeedCommand.resize(0);
	m_vdVerticalCommand.resize(0);
	m_vdTurnCommand.resize(0);

    m_vdHistories_X.resize(m_iNObj);
    m_vdHistories_Y.resize(m_iNObj);
	m_vbLastMeasValid.resize(m_iNObj);
    /* there is an X and Y history for each object.  They need to be
       initially zero in length so that we can fill them up with real
       data as it becomes available. */
    for(int i = 0; i < m_iNObj; i++)
    {
        m_vdHistories_X[i].resize(0);
        m_vdHistories_Y[i].resize(0); 
		m_vbLastMeasValid[i] = false;
    }


    /* sets up the frames that are available in the "view" menu */
    m_pTrackerFrameGroup = new MT_TrackerFrameGroup();
    //m_pTrackerFrameGroup->pushFrame(&m_pDiffFrames[0],      "Diff Frame");
	m_pTrackerFrameGroup->pushFrame(&m_pUndistortedFrames[0], "Undistorted");
    m_pTrackerFrameGroup->pushFrame(&m_pThreshFrames[0],    "Threshold Frame");
	m_pTrackerFrameGroup->pushFrame(&m_pHSVFrames[0], "HSV");
	m_pTrackerFrameGroup->pushFrame(&m_pHFrames[0], "H");
	m_pTrackerFrameGroup->pushFrame(&m_pSFrames[0], "S");
	m_pTrackerFrameGroup->pushFrame(&m_pVFrames[0], "V");
	m_pTrackerFrameGroup->pushFrame(&m_pGSFrames[0], "GS");

    for(unsigned int i = 0; i < 3; i++)
    {
        m_pAuxFrameGroups[i] = new MT_TrackerFrameGroup();
		m_pAuxFrameGroups[i]->pushFrame(&m_pUndistortedFrames[i+1], "Undistorted");
        m_pAuxFrameGroups[i]->pushFrame(&m_pThreshFrames[i+1], "Threshold Frame");
        m_pAuxFrameGroups[i]->pushFrame(&m_pHSVFrames[i+1], "HSV");
        m_pAuxFrameGroups[i]->pushFrame(&m_pHFrames[i+1], "H");
        m_pAuxFrameGroups[i]->pushFrame(&m_pSFrames[i+1], "S");
        m_pAuxFrameGroups[i]->pushFrame(&m_pVFrames[i+1], "V");
        m_pAuxFrameGroups[i]->pushFrame(&m_pGSFrames[i+1], "GS");        
    }

    /* Data group and Data report setup.
     *
     * This is how the tracker class interacts with the GUI.  When the
     * tracker is initialized, the GUI pulls the contents of
     * m_vDataGroups and automatically creates dialog boxes enabling
     * the user to view/edit these values.
     *
     * An MT_DataGroup is a list of parameters and can be read/write -
     * e.g. minimum blob size, or drawing color.
     * An MT_DataReport is a list of vectors of numbers and is
     * read-only.  e.g. a report of the found blobs positions.
     */
    
    /* set up the parameter groups for parameter modification, etc. */
    /* first group is for blob tracking parameters */
    MT_DataGroup* dg_blob = new MT_DataGroup("Blob Tracking Parameters");
	dg_blob->AddUInt("HSV V Threshold", &m_iVThresh, MT_DATA_READWRITE, 0, 255);
	dg_blob->AddUInt("HSV H Threshold Low", &m_iHThresh_Low, MT_DATA_READWRITE, 0, 255);
	dg_blob->AddUInt("HSV H Threshold High", &m_iHThresh_High, MT_DATA_READWRITE, 0, 255);
	dg_blob->AddUInt("HSV S Threshold Low", &m_iSThresh_Low, MT_DATA_READWRITE, 0, 255);
	dg_blob->AddUInt("HSV S Threshold High", &m_iSThresh_High, MT_DATA_READWRITE, 0, 255);
    dg_blob->AddDouble("Water Depth", &m_dWaterDepth, MT_DATA_READWRITE, 0);
    dg_blob->AddUInt("Difference Threshold", /* parameter name */
                     &m_iBlobValThresh,      /* pointer to variable */
                     MT_DATA_READWRITE,      /* read-only or not */
                     0,                      /* minimum value */
                     255);                   /* maximum value */
    dg_blob->AddUInt("Min Blob Area",
                    &m_iBlobAreaThreshLow,
                    MT_DATA_READWRITE,
                    0);
    dg_blob->AddUInt("Max Blob Area",
                    &m_iBlobAreaThreshHigh,
                    MT_DATA_READWRITE,
                    0);
	dg_blob->AddUInt("Search Area Padding",
		             &m_iSearchAreaPadding,
					 MT_DATA_READWRITE,
					 0);
    dg_blob->AddDouble("Position Disturbance Sigma",
                       &m_dSigmaPosition,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Heading Disturbance Sigma",
                       &m_dSigmaHeading,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Speed Disturbance Sigma",
                       &m_dSigmaSpeed,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Position Measurement Sigma",
                       &m_dSigmaPositionMeas,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Heading Measurement Sigma",
                       &m_dSigmaHeadingMeas,
                       MT_DATA_READWRITE,
                       0);

    MT_DataGroup* dg_draw = new MT_DataGroup("Drawing Options");
    dg_draw->AddBool("Show blob arrows", &m_bShowBlobs);
    dg_draw->AddBool("Show tracking arrows", &m_bShowTracking);
    
    /* now stuff the parameter groups into m_vDataGroups, which
     * MT_TrackerBase will automagically report to the GUI */
    m_vDataGroups.resize(0);
    m_vDataGroups.push_back(dg_blob);
    m_vDataGroups.push_back(dg_draw);
    
    MT_DataReport* dr_tracked = new MT_DataReport("Tracked data");
    dr_tracked->AddDouble("X", &m_vdTracked_X);
    dr_tracked->AddDouble("Y", &m_vdTracked_Y);
    dr_tracked->AddDouble("Hdg", &m_vdTracked_Heading);
    dr_tracked->AddDouble("Spd", &m_vdTracked_Speed);

	MT_DataReport* dr_blobs = new MT_DataReport("Blob Info");
	dr_blobs->AddDouble("X", &m_vdBlobs_X);
	dr_blobs->AddDouble("Y", &m_vdBlobs_Y);
	dr_blobs->AddDouble("Area", &m_vdBlobs_Area);

    m_vDataReports.resize(0);
    m_vDataReports.push_back(dr_blobs);
	m_vDataReports.push_back(dr_tracked);

}

void BelugaTracker::doTrain(IplImage* frame)
{
	printf("Training.\n");
	m_iFrameWidth = frame->width;
	m_iFrameHeight = frame->height;
	CvSize fsize = cvSize(m_iFrameWidth, m_iFrameHeight);

	MT_TrackerBase::doTrain(frame);

	//HSVSplit(frame);

	//cvCopy(m_pVFrame, BG_frame);

	/*if(m_pGSThresholder)
	{
		delete m_pGSThresholder;
	}*/

	//m_pGSThresholder = new MT_GSThresholder(BG_frame);
    /* The thresholder manages these frames, but by grabbing pointers
       to them we can pass them to the GUI. */
	/*m_pGSFrame = m_pGSThresholder->getGSFrame();
    m_pDiffFrame = m_pGSThresholder->getDiffFrame();
    m_pThreshFrame = m_pGSThresholder->getThreshFrame();*/
}

/* This gets called by MT_TrackerBase::doInit.  I use it here more to
 * allocate other bits of memory - a lot of this could actually go
 * into doInit, but there's no real harm */
void BelugaTracker::createFrames()
{
    /* this makes sure that the BG_frame is created */
    MT_TrackerBase::createFrames();

	if(m_pHSVFrames[0])
	{
		for(int i = 0; i < 4; i++)
		{
			//cvReleaseImage(&m_pGSFrames[i]);
			cvReleaseImage(&m_pUndistortedFrames[i]);
			cvReleaseImage(&m_pHSVFrames[i]);
			cvReleaseImage(&m_pHFrames[i]);
			cvReleaseImage(&m_pSFrames[i]);
			cvReleaseImage(&m_pVFrames[i]);
			cvReleaseImage(&m_pThreshFrames[i]);
		}
		cvReleaseImage(&m_pTempFrame1);
		cvReleaseImage(&m_pTempFrame2);
	}

	if(m_pUndistortMapX)
	{
		cvReleaseImage(&m_pUndistortMapX);
		cvReleaseImage(&m_pUndistortMapY);
	}

	for(int i = 0; i < 4; i++)
	{
		//m_pGSFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);
		m_pUndistortedFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
			IPL_DEPTH_8U, 3);
		m_pThreshFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                           IPL_DEPTH_8U, 1);
		m_pHSVFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                        IPL_DEPTH_8U, 3);
		m_pHFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
		m_pSFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
		m_pVFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
	}
	m_pTempFrame1 = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);
	m_pTempFrame2 = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);

	m_pUndistortMapX = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_32F, 1);
	m_pUndistortMapY = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_32F, 1);

	//m_pGSThresholder = NULL;
    /* Create the Thresholder and Blobber objects */
    //m_pGSThresholder = new MT_GSThresholder(BG_frame);
    //m_pGYBlobber = new GYBlobber(m_iNObj);
    /* Initialize the Hungarian Matcher */
    m_HungarianMatcher.doInit(m_iNObj);

    /* Initialize some of the matrices used for tracking */
    m_pQ = cvCreateMat(5, 5, CV_64FC1);
    m_pR = cvCreateMat(4, 4, CV_64FC1);
    cvSetIdentity(m_pQ);
    cvSetIdentity(m_pR);
    m_px0 = cvCreateMat(5, 1, CV_64FC1);
    m_pz = cvCreateMat(4, 1, CV_64FC1);

    /* Create the UKF objects */
    m_vpUKF.resize(m_iNObj);
    for(int i = 0; i < m_iNObj; i++)
    {
        m_vpUKF[i] = MT_UKFInit(5, 4, 0.1); /* 5 states, 4
                                        * measurements, 0.1 is a parameter */
    }

} /* endof createFrames */

/* this gets called during the destructor */
void BelugaTracker::releaseFrames()
{
    
    /* Nothing really to do here.  BG_Frame is managed by the base
     * class and the GS, Diff, and Thresh frames are managed by
     * m_pGSThresholder */
}

/* Initialize a data file for output.  Gets called from the GUI or
 * command line */
void BelugaTracker::initDataFile()
{
    /* An XDF can store parameters in XML format and manages data
       output to other files that are keyed in the master XML file.

       These can then be read by e.g. MATLAB */
    char v[10];
    sprintf(v, "%d", m_iStartFrame);
    m_XDF.writeParameterToXML("Starting Frame", v);
    sprintf(v, "%d", m_iStopFrame);
    m_XDF.writeParameterToXML("Stopping Frame", v);

    m_XDF.addDataStream("Blob X", "blob_x.dat");
    m_XDF.addDataStream("Blob Y", "blob_y.dat");
    m_XDF.addDataStream("Blob Area", "blob_area.dat");
    m_XDF.addDataStream("Blob Orientation", "blob_orientation.dat");
    m_XDF.addDataStream("Blob Major Axis", "blob_major.dat");
    m_XDF.addDataStream("Blob Minor Axis", "blob_minor.dat");

    m_XDF.addDataStream("Time", "time_stamp.dat");

    m_XDF.addDataStream("Tracked X", "tracked_x.dat");
    m_XDF.addDataStream("Tracked Y", "tracked_y.dat");  
    m_XDF.addDataStream("Tracked Z", "tracked_z.dat");  
    m_XDF.addDataStream("Tracked Heading", "tracked_heading.dat");    
    m_XDF.addDataStream("Tracked Speed", "tracked_speed.dat");

	m_XDF.addDataStream("Tracked Camera X", "tracked_camera_x.dat");
	m_XDF.addDataStream("Tracked Camera Y", "tracked_camera_y.dat");

	m_XDF.addDataStream("Depth Measurement", "depth_meas.dat");
	m_XDF.addDataStream("Speed Command", "cmd_speed.dat");
    m_XDF.addDataStream("Vertical Command", "cmd_vert.dat");
    m_XDF.addDataStream("Turn Command", "cmd_turn.dat");
    
    MT_TrackerBase::initDataFile();
}

/* a helper function to write data to a file */
void BelugaTracker::writeData()
{
    /* the XDF object handles writing data to the right files - all we
       have to do is pass the data as vectors */
	std::vector<double> tmp;
	tmp.resize(0);
	tmp.push_back(m_dCurrentTime);
    m_XDF.writeData("Time"             , tmp);

    m_XDF.writeData("Blob X"           , m_vdBlobs_X); 
    m_XDF.writeData("Blob Y"           , m_vdBlobs_Y); 
    m_XDF.writeData("Blob Area"        , m_vdBlobs_Area); 
    m_XDF.writeData("Blob Orientation" , m_vdBlobs_Orientation); 
    m_XDF.writeData("Blob Major Axis"  , m_vdBlobs_MajorAxis); 
    m_XDF.writeData("Blob Minor Axis"  , m_vdBlobs_MinorAxis); 

    m_XDF.writeData("Tracked X"        , m_vdTracked_X); 
    m_XDF.writeData("Tracked Y"        , m_vdTracked_Y); 
    m_XDF.writeData("Tracked Z"        , m_vdTracked_Z); 
    m_XDF.writeData("Tracked Heading"  , m_vdTracked_Heading); 
    m_XDF.writeData("Tracked Speed"    , m_vdTracked_Speed); 

	std::vector<double> tmp2;
	tmp.resize(0);
	tmp2.resize(0);
	for(unsigned int i = 0; i < m_vdaTracked_XC[0].size(); i++)
	{
		tmp.push_back(m_vdaTracked_XC[0][i]);
		tmp.push_back(m_vdaTracked_XC[1][i]);
		tmp.push_back(m_vdaTracked_XC[2][i]);
		tmp.push_back(m_vdaTracked_XC[3][i]);

		tmp2.push_back(m_vdaTracked_YC[0][i]);
		tmp2.push_back(m_vdaTracked_YC[1][i]);
		tmp2.push_back(m_vdaTracked_YC[2][i]);
		tmp2.push_back(m_vdaTracked_YC[3][i]);
	}
	m_XDF.writeData("Tracked Camera X", tmp);
	m_XDF.writeData("Tracked Camera Y", tmp2);
    
	m_XDF.writeData("Depth Measurement", m_vdDepthMeasurement);
	m_XDF.writeData("Speed Command", m_vdSpeedCommand);
	m_XDF.writeData("Vertical Command", m_vdVerticalCommand);
	m_XDF.writeData("Turn Command", m_vdTurnCommand);

}

void BelugaTracker::setMasks(const char* maskfile1,
		const char* maskfile2,
		const char* maskfile3,
		const char* maskfile4)
{
	if(!maskfile1 || !maskfile2 || !maskfile3 || !maskfile4)
	{
		return;
	}

	const char* files[4] = {maskfile1, maskfile2, maskfile3, maskfile4};

	for(int i = 0; i < 4; i++)
	{
		if(m_pMasks[i])
		{
			cvReleaseImage(&m_pMasks[i]);
		}

		m_pMasks[i] = cvLoadImage(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		if(m_pMasks[i] == 0)
		{
			fprintf(stderr, "BelugaTracker Error:  Unable to load mask file %s\n", files[i]);
		}
		else
		{
			if(m_pMasks[i]->width != m_pHSVFrames[i]->width || m_pMasks[i]->height != m_pHSVFrames[i]->height)
			{
				fprintf(stderr, "BelugaTracker Error:  Mask file %s has the wrong size\n", files[i]);
				m_pMasks[i] = NULL;
			}
		}
	}
}

void BelugaTracker::setCalibrations(const char* calibfile1,
		const char* calibfile2,
		const char* calibfile3,
		const char* calibfile4)
{
	if(!calibfile1 || !calibfile2 || !calibfile3 || !calibfile4)
	{
		return;
	}

	const char* files[4] = {calibfile1, calibfile2, calibfile3, calibfile4};

	for(int i = 0; i < 4; i++)
	{
		Beluga_CalibrationDataFile f(files[i]);
		if(!f.didLoadOK())
		{
			fprintf(stderr, "BelugaTracker Error:  Unable to load calibration %s", files[i]);
		}
		else
		{
			CalibrationData d;
			f.getCalibration(&d);

			if(!m_pCameraMatrices[i])
			{
				m_pCameraMatrices[i] = cvCreateMat(3, 3, CV_32FC1);
				m_pDistortionCoeffs[i] = cvCreateMat(5, 1, CV_32FC1);
				m_pRotationVectors[i] = cvCreateMat(3, 1, CV_32FC1);
				m_pRotationMatrices[i] = cvCreateMat(3, 3, CV_32FC1);
				m_pTranslationVectors[i] = cvCreateMat(3, 1, CV_32FC1);
			}

            m_CoordinateTransforms[i].setCalibrationAndWaterDepth(d, m_dWaterDepth);

			if(!calibrationDataToOpenCVCalibration(d,
				m_pCameraMatrices[i],
				m_pDistortionCoeffs[i],
				m_pRotationVectors[i],
				m_pTranslationVectors[i],
				m_pRotationMatrices[i]))
			{
				fprintf(stderr, "BelugaTracker Error:  "
                        "Unable to copy calibration from %s", files[i]);
				cvReleaseMat(&m_pCameraMatrices[i]);
				cvReleaseMat(&m_pDistortionCoeffs[i]);
				cvReleaseMat(&m_pRotationVectors[i]);
				cvReleaseMat(&m_pRotationMatrices[i]);
				cvReleaseMat(&m_pTranslationVectors[i]);
				m_pCameraMatrices[i] = NULL;
				m_pDistortionCoeffs[i] = NULL;
				m_pRotationVectors[i] = NULL;
				m_pTranslationVectors[i] = NULL;
				m_pRotationMatrices[i] = NULL;
			}

		}
	}

	if(m_pCameraMatrices[0])
	{
		cvInitUndistortMap(m_pCameraMatrices[0], 
			m_pDistortionCoeffs[0],
			m_pUndistortMapX,
			m_pUndistortMapY);
	}
}

void BelugaTracker::HSVSplit(IplImage* frame, int i)
{
	cvCvtColor(frame, m_pHSVFrames[i], CV_RGB2HSV);
	cvSplit(m_pHSVFrames[i], 
		m_pHFrames[i], 
		m_pSFrames[i], 
		m_pVFrames[i],
		NULL);
}

/* Main tracking function - gets called by MT_TrackerFrameBase every
 * time step when the application is not paused. */
void BelugaTracker::doTracking(IplImage* frame)
{
	std::cerr << "ERROR:  This function should never get called!  BelugaTracker::doTracking(IplImage* frame)\n";
}

void BelugaTracker::doTracking(IplImage* frames[4])
{
    /* time-keeping, if necessary
     * NOTE this is not necessary for keeping track of frame rate */
    static double t_prev = MT_getTimeSec();
    double t_now = MT_getTimeSec();
    m_dDt = t_now - t_prev;
    t_prev = t_now;
	m_dCurrentTime = t_now;

    /* keeping track of the frame number, if necessary */
    m_iFrameCounter++;

//	printf("Tracking: Update parameters\n");
    /* This checks every time step to see if the UKF parameters have
       changed and modifies the UKF structures accordingly.  This will
       also get called the first time through b/c the "Prev" values get
       set to zero initially.   There may be a more efficient way to do
       this, but because the values need to be embedded into the CvMat
       objects I'm not sure how else to do it. */ 
    if(
        m_dSigmaPosition != m_dPrevSigmaPosition ||
        m_dSigmaHeading != m_dPrevSigmaHeading ||
        m_dSigmaSpeed != m_dPrevSigmaSpeed ||
        m_dSigmaPositionMeas != m_dPrevSigmaPositionMeas ||
        m_dSigmaHeadingMeas != m_dPrevSigmaHeadingMeas
        )
    {
        /* these are the diagonal entries of the "Q" matrix, which
           represents the variances of the process noise.  They're
           modeled here as being independent and uncorrellated. */
        cvSetReal2D(m_pQ, 0, 0, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, 1, 1, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, 2, 2, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, 3, 3, m_dSigmaHeading*m_dSigmaHeading);
        cvSetReal2D(m_pQ, 4, 4, m_dSigmaSpeed*m_dSigmaSpeed);        

        /* these are the diagonal entries of the "R matrix, also
           assumed to be uncorrellated. */
        cvSetReal2D(m_pR, 0, 0, m_dSigmaPositionMeas*m_dSigmaPositionMeas);
        cvSetReal2D(m_pR, 1, 1, m_dSigmaPositionMeas*m_dSigmaPositionMeas);
        cvSetReal2D(m_pR, 3, 3, m_dSigmaHeadingMeas*m_dSigmaHeadingMeas);
        cvSetReal2D(m_pR, 2, 2, m_dSigmaPositionMeas*m_dSigmaPositionMeas);

//		printf("a.1\n");
		/* makes sure we only have to copy the numbers once - it will
		 * automatically get copied again later if necessary */
		adjustRMatrixAndZForMeasurementSize(m_pR, m_pz, 1);
		for(int i = 0; i < m_iNObj; i++)
		{
			m_vpUKF[i]->m = m_pR->rows;
		}
//		printf("a.2\n");

		m_dPrevSigmaPosition = m_dSigmaPosition;
		m_dPrevSigmaHeading = m_dSigmaHeading;
		m_dPrevSigmaSpeed = m_dSigmaSpeed;
		m_dPrevSigmaPositionMeas = m_dSigmaPositionMeas;
		m_dPrevSigmaHeadingMeas = m_dSigmaHeadingMeas;

        /* this step actually copies the Q and R matrices to the UKF
           and makes sure that it's internals are properly initialized -
           it's set up to handle the fact that the sizes of these
           matrices could have changed. */
//		printf("a.3\n");
        for(int i = 0; i < m_iNObj; i++)
        {
            MT_UKFCopyQR(m_vpUKF[i], m_pQ, m_pR);
        }
    }

//	printf("b\n");
	std::vector<std::vector<double> > measX;
	std::vector<std::vector<double> > measY;
	std::vector<std::vector<double> > measZ;

	measX.resize(m_iNObj);
	measY.resize(m_iNObj);
	measZ.resize(m_iNObj);
	for(int i = 0; i < m_iNObj; i++)
	{
		measX[i].resize(0);
		measY[i].resize(0);
		measZ[i].resize(0);
	}

//	printf("Tracking:  Update search rectangles\n");

	/* determine search rectangles */
	for(int i = 0; i < 4; i++)
	{
        m_CoordinateTransforms[i].setWaterDepth(m_dWaterDepth);

		m_SearchIndexes[i].resize(0);
		m_SearchArea[i].resize(0);
		std::vector<unsigned int> search_indexes_this_rect;
		search_indexes_this_rect.resize(1);

		for(int j = 0; j < m_iNObj; j++)
		{
			search_indexes_this_rect[0] = j;

			if(!m_vbLastMeasValid[j] || m_vdHistories_X[j].size() == 0)
			{
				m_SearchIndexes[i].push_back(search_indexes_this_rect);
				m_SearchArea[i].push_back(cvRect(0, 0, m_iFrameWidth, m_iFrameHeight));
				continue;
			}

			double x = m_vdTracked_X[j];
			double y = m_vdTracked_Y[j];
			double z = m_vdTracked_Z[j];
			double u = 0;
			double v = 0;
 
			m_CoordinateTransforms[i].worldToImage(x, y, z, &u, &v, false);
			//v = m_iFrameHeight - v;
//			printf("%f, %f, %f -> %f, %f\n", x, y, z, u, v);

			m_vdaTracked_XC[i][j] = u;
			m_vdaTracked_YC[i][j] = v;

			if(u > -(double)m_iSearchAreaPadding && u < (double)(m_iFrameWidth+m_iSearchAreaPadding)
				&& v > -(double)m_iSearchAreaPadding && v < (double)(m_iFrameHeight+m_iSearchAreaPadding))
			{
				m_SearchIndexes[i].push_back(search_indexes_this_rect);
				m_SearchArea[i].push_back(searchRectAtPointWithSize(u,v,m_iSearchAreaPadding));
			}
		}

//		printf("============= IN %d ============\n", i);
/*		printf("%d search areas\n", m_SearchArea[i].size());
		for(unsigned int j = 0; j < m_SearchArea[i].size(); j++)
		{
			printf("[%d, %d, %d, %d] including indeces ", 
				m_SearchArea[i][j].x,
				m_SearchArea[i][j].y,
				m_SearchArea[i][j].width,
				m_SearchArea[i][j].height);
			for(unsigned int k = 0; k < m_SearchIndexes[i][j].size(); k++)
			{
				printf("%d ", m_SearchIndexes[i][j][k]);
			}
			printf("\n");
		}*/

	    combineSearchAreas(&m_SearchArea[i], &m_SearchIndexes[i]);

//		printf("============= OUT %d ============\n", i);
/*		printf("%d search areas\n", m_SearchArea[i].size());
		for(unsigned int j = 0; j < m_SearchArea[i].size(); j++)
		{
			printf("[%d, %d, %d, %d] including indeces ", 
				m_SearchArea[i][j].x,
				m_SearchArea[i][j].y,
				m_SearchArea[i][j].width,
				m_SearchArea[i][j].height);
			for(unsigned int k = 0; k < m_SearchIndexes[i][j].size(); k++)
			{
				printf("%d ", m_SearchIndexes[i][j][k]);
			}
			printf("\n");
		}*/

	}

//	printf("Tracking:  Image processing\n");

	/* image processing and blob finding */
	for(int i = 0; i < 4; i++)
	{
		cvRemap(frames[i], m_pUndistortedFrames[i], m_pUndistortMapX, m_pUndistortMapY);
		HSVSplit(m_pUndistortedFrames[i], i);

		//HSVSplit(frames[i], i);

		/* just for display */
		m_pGSFrames[i] = m_pVFrames[i];

		cvThreshold(m_pVFrames[i], m_pThreshFrames[i], m_iVThresh, 255, CV_THRESH_BINARY_INV);
		cvThreshold(m_pHFrames[i], m_pTempFrame1, m_iHThresh_High, 255, CV_THRESH_BINARY);
		cvThreshold(m_pHFrames[i], m_pTempFrame2, m_iHThresh_Low, 255, CV_THRESH_BINARY_INV);
		cvOr(m_pTempFrame1, m_pTempFrame2, m_pTempFrame1);
		cvAnd(m_pTempFrame1, m_pThreshFrames[i], m_pThreshFrames[i]);
		cvThreshold(m_pSFrames[i], m_pTempFrame1, m_iSThresh_Low, 255, CV_THRESH_BINARY);
		cvThreshold(m_pSFrames[i], m_pTempFrame2, m_iSThresh_High, 255, CV_THRESH_BINARY_INV);
		cvAnd(m_pTempFrame1, m_pTempFrame2, m_pTempFrame1);
		cvAnd(m_pTempFrame1, m_pThreshFrames[i], m_pThreshFrames[i]);
		cvSmooth(m_pThreshFrames[i], m_pThreshFrames[i], CV_MEDIAN, 3);
		if(m_pMasks[i])
		{
			cvAnd(m_pThreshFrames[i], m_pMasks[i], m_pThreshFrames[i]);
		}

		m_vBlobs[i] = m_YABlobber.FindBlobs(m_pThreshFrames[i],
			5, /* min perimeter */
			m_iBlobAreaThreshLow,
			NO_MAX,
			m_iBlobAreaThreshHigh);
		for(unsigned int j = 0; j < m_vBlobs[i].size(); j++)
		{
			m_vBlobs[i][j].COMy = m_iFrameHeight - m_vBlobs[i][j].COMy;
		}

        m_CoordinateTransforms[i].setWaterDepth(m_dWaterDepth);
	}

//	printf("Tracking:  Assign Measurements\n");
	/* measurement association */
	for(int i = 0; i < m_iNObj; i++)
	{
		m_vvdMeas_X[i].resize(0);
		m_vvdMeas_Y[i].resize(0);
		m_vviMeas_A[i].resize(0);
		m_vviMeas_Cam[i].resize(0);
		m_vvdMeas_Hdg[i].resize(0);
	}
	for(int i = 0; i < 4; i++)
	{
		for(unsigned int j = 0; j < m_SearchArea[i].size(); j++)
		{
//			printf("Tracking:   Collect Blobs\n");
			/* blobs in search area */
			std::vector<YABlob> blobs_this_rect;
			blobs_this_rect.resize(0);
			for(unsigned int k = 0; k < m_vBlobs[i].size(); k++)
			{
				if(pointInCvRect(m_vBlobs[i][k].COMx, m_vBlobs[i][k].COMy, m_SearchArea[i][j]))
				{
					blobs_this_rect.push_back(m_vBlobs[i][k]);
				}
			}

			if(blobs_this_rect.size() == 0)
			{
				continue;
			}

//			printf("Tracking:   Setup matcher\n");
			MT_HungarianMatcher matcher;
			bool do_match = false;
			if(blobs_this_rect.size() >= m_SearchIndexes[i][j].size() && blobs_this_rect.size() > 1)
			{
				do_match = true;
				matcher.doInit(m_SearchIndexes[i][j].size(), blobs_this_rect.size());
			}

//			printf("Tracking:   Distance Loop (doMatch?  %s)\n", do_match ? "yes" : "no");
			//for(unsigned int k = 0; k < m_SearchIndexes[i][j].size(); k++)
			for(unsigned int m = 0; m < blobs_this_rect.size(); m++)
			{
//				printf("\t\t\tm = %d\n", m);
				double d2;
				double d2min = 1e10;
				int kmin = -1;
				/* i'th camera, j'th rectangle, k'th index */
				//for(unsigned int m = 0; m < blobs_this_rect.size(); m++)
				for(unsigned int k = 0; k < m_SearchIndexes[i][j].size(); k++)
				{
//					printf("\t\t\tk = %d\n", k);
					unsigned int n = m_SearchIndexes[i][j][k];
					double xk = m_vdaTracked_XC[i][n];
					double yk = m_vdaTracked_YC[i][n];

//					printf("\t\t\t   1\n");
					d2 = (blobs_this_rect[m].COMx - xk)*(blobs_this_rect[m].COMx - xk)
						+ (blobs_this_rect[m].COMy - yk)*(blobs_this_rect[m].COMy - yk);
//					printf("d2 = %f\n", d2);

					if(do_match)
					{
						matcher.setValue(k, m, d2);
					}
//					printf("\t\t\t   2\n");
					if((!m_vbLastMeasValid[n] || 
						(d2 < DEFAULT_GATE_DIST2) || 
						(m_vdHistories_X[n].size() == 0)) 
						&& d2 < d2min)
					{
						kmin = k;
						d2min = d2;
					}
//					printf("\t\t\t   3\n");
				}

//				printf("\t\t\t    A\n");

				if(kmin < 0)
				{
					kmin = 0;
				}

				unsigned int n = m_SearchIndexes[i][j][kmin];

//				printf("\t\t\t    B\n");
				if(kmin >= 0 && !do_match  && m_vvdMeas_X[n].size() == 0)
				{
//					printf("\t\t\tForce assign\n");
					m_vvdMeas_X[n].push_back(blobs_this_rect[m].COMx);
					m_vvdMeas_Y[n].push_back(blobs_this_rect[m].COMy);
					m_vvdMeas_Hdg[n].push_back(blobs_this_rect[m].orientation);
					m_vviMeas_A[n].push_back(blobs_this_rect[m].area);
					m_vviMeas_Cam[n].push_back(i);
//					printf("n = %d, i = %d\n", n, i); 
				}

//				printf("loop out\n");
			}

//			printf("Tracking:   Do match\n");
			if(do_match)
			{
			std::vector<int> matches;
			matches.resize(m_SearchIndexes[i][j].size());
			matcher.doMatch(&matches);

//			printf("Hungarian matches:  ");
			for(unsigned int k = 0; k < matches.size(); k++)
			{
//				printf("%d -> %d ", k, matches[k]);
				unsigned int n = m_SearchIndexes[i][j][k];
				m_vvdMeas_X[n].push_back(blobs_this_rect[matches[k]].COMx);
				m_vvdMeas_Y[n].push_back(blobs_this_rect[matches[k]].COMy);
				m_vvdMeas_Hdg[n].push_back(blobs_this_rect[matches[k]].orientation);
				m_vviMeas_A[n].push_back(blobs_this_rect[matches[k]].area);
				m_vviMeas_Cam[n].push_back(i);
			}
//			printf("\n");
			}

		}
	}

//	printf("Tracking:  UKF Update\n");

	/* filtering */
	for(int i =0; i < m_iNObj; i++)
	{
		unsigned int nmeas = m_vvdMeas_X[i].size();

//		printf("Got %d Meas's for Object %d: ", m_vvdMeas_X[i].size(), i);
/*		for(unsigned int j = 0; j < m_vvdMeas_X[i].size(); j++)
		{
			printf("(%f, %f, %f in Q%d) ", m_vvdMeas_X[i][j], m_vvdMeas_Y[i][j], m_vvdMeas_Hdg[i][j], m_vviMeas_Cam[i][j]);
		}
		printf("\n");
*/

//		printf("A\n");
		if(nmeas)
		{
			adjustRMatrixAndZForMeasurementSize(m_pR, m_pz, nmeas);
			MT_UKFCopyQR(m_vpUKF[i], m_pQ, m_pR);
			m_vpUKF[i]->m = m_pR->rows;
		}

        bool valid_meas = m_iFrameCounter > 1 && nmeas > 0;

//		printf("B\n");
        /* if any state is NaN, reset the UKF
         * This shouldn't happen anymore, but it's a decent safety
         * check.  It could probably be omitted if we want to
         * optimize for speed... */
        if(m_iFrameCounter > 1 &&
           (!CvMatIsOk(m_vpUKF[i]->x) ||
            !CvMatIsOk(m_vpUKF[i]->P)))
        {
//			printf("B.1\n");
            MT_UKFFree(&(m_vpUKF[i]));
//			printf("B.2 %d\n", m_pR->rows);
            m_vpUKF[i] = MT_UKFInit(5, m_pR->rows, 0.1);
//			printf("B.3\n");
            MT_UKFCopyQR(m_vpUKF[i], m_pQ, m_pR);
//			printf("B.4\n");
            valid_meas = false;
        }

//		printf("C\n");

		m_vbLastMeasValid[i] = valid_meas;

        /* if we're going to accept this measurement */
        if(valid_meas)
        {
            /* UKF prediction step, note we use function pointers to
               the beluga_dynamics and beluga_measurement functions defined
               above.  The final parameter would be for the control input
               vector, which we don't use here so we pass a NULL pointer */
//			printf("UKF Predict\n");
            MT_UKFPredict(m_vpUKF[i],
                          &beluga_dynamics,
                          &beluga_measurement,
                          NULL);

//			printf("Meas assign\n");
			/* build the measurement vector */
			double x, y, z, th;
			double th_prev = cvGetReal2D(m_vpUKF[i]->x, 3, 0);
			unsigned int cam_no;
			for(unsigned int j = 0; j < nmeas; j++)
			{
				cam_no = m_vviMeas_Cam[i][j];
				m_CoordinateTransforms[cam_no].imageAndDepthToWorld(m_vvdMeas_X[i][j],
					m_vvdMeas_Y[i][j],
					0, &x, &y, &z, false);
				th = rectifyAngleMeasurement(m_vvdMeas_Hdg[i][j],
					m_vdHistories_X[i],
					m_vdHistories_Y[i],
					N_hist,
					th_prev);
//				printf("%f, %f -> %f\n", m_vvdMeas_Hdg[i][j], MT_RAD2DEG*th_prev, MT_RAD2DEG*th);
				m_vvdMeas_Hdg[i][j] = MT_RAD2DEG*th;
				cvSetReal2D(m_pz, j*3 + 0, 0, x);
				cvSetReal2D(m_pz, j*3 + 1, 0, y);
				cvSetReal2D(m_pz, j*3 + 2, 0, th);
			}
			cvSetReal2D(m_pz, m_pz->rows - 1, 0, m_dWaterDepth);

//			printf("UKF SetMeas\n");
//			printf("z = [");
/*			for(unsigned int r = 0; r < m_pz->rows; r++)
			{
				printf("%f ", cvGetReal2D(m_pz, r, 0));
			}
			printf("]\n");
*/
			MT_UKFSetMeasurement(m_vpUKF[i], m_pz);
//			printf("UKF Correct\n");
			MT_UKFCorrect(m_vpUKF[i]);

/*			printf("x = [");
			for(unsigned int r = 0; r < m_vpUKF[i]->x->rows; r++)
			{
				printf("%f ", cvGetReal2D(m_vpUKF[i]->x, r, 0));
			}
			printf("]\n");
*/
//			printf("constrain\n");
			constrain_state(m_vpUKF[i]->x, m_vpUKF[i]->x1, 10.0, m_dWaterDepth);
/*			printf("x = [");
			for(unsigned int r = 0; r < m_vpUKF[i]->x->rows; r++)
			{
				printf("%f ", cvGetReal2D(m_vpUKF[i]->x, r, 0));
			}
			printf("]\n");
*/

		}
		else
		{
			// on the first frame, take the average of the measurements
			if(m_iFrameCounter == 1)
			{
				if(nmeas > 0)
				{
					double xavg = 0;
					double yavg = 0;
					double qx = 0;
					double qy = 0;
					double x, y, z;
					unsigned int cam_no;
					for(unsigned int j = 0; j < nmeas; j++)
					{
						cam_no = m_vviMeas_Cam[i][j];
						m_CoordinateTransforms[cam_no].imageAndDepthToWorld(m_vvdMeas_X[i][j],
							m_vvdMeas_Y[i][j],
							0, &x, &y, &z, false);
						xavg += x;
						yavg += y;
						qx += cos(MT_DEG2RAD*m_vvdMeas_Hdg[i][j]);
						qy += sin(MT_DEG2RAD*m_vvdMeas_Hdg[i][j]);
					}
					xavg /= (double) nmeas;
					yavg /= (double) nmeas;
					double th = atan2(qy, qx);
//					printf("Should be initializing %d at %f %f %f %f 0\n", i, xavg, yavg, m_dWaterDepth, th);
					cvSetReal2D(m_vpUKF[i]->x, 0, 0, xavg);
					cvSetReal2D(m_vpUKF[i]->x, 1, 0, yavg);
					cvSetReal2D(m_vpUKF[i]->x, 2, 0, m_dWaterDepth);
					cvSetReal2D(m_vpUKF[i]->x, 3, 0, th);
					cvSetReal2D(m_vpUKF[i]->x, 4, 0, 0);
					m_vbLastMeasValid[i] = true;
				}
			}
			else
			{
				// use the prediction
				cvCopy(m_vpUKF[i]->x1, m_vpUKF[i]->x);
			}
		}

//		printf("Tracking: State update\n");

		rollHistories(&m_vdHistories_X[i], 
			&m_vdHistories_Y[i], 
			cvGetReal2D(m_vpUKF[i]->x, 0, 0),
			cvGetReal2D(m_vpUKF[i]->x, 1, 0),
			N_hist);

		/* grab the state estimate and store it in variables that will
           make it convenient to save it to a file. */
        CvMat* x = m_vpUKF[i]->x;

        m_vdTracked_X[i] = cvGetReal2D(x, 0, 0);
        m_vdTracked_Y[i] = cvGetReal2D(x, 1, 0);
        m_vdTracked_Z[i] = cvGetReal2D(x, 2, 0);
        m_vdTracked_Heading[i] = cvGetReal2D(x, 3, 0);
        m_vdTracked_Speed[i] = cvGetReal2D(x, 4, 0);


	}

	writeData();

}

std::vector<double> BelugaTracker::getBelugaState(unsigned int i)
{
	std::vector<double> r;
	r.resize(0);

	if((int) i >= m_iNObj || i >= m_vdTracked_X.size())
	{
		fprintf(stderr, "BelugaTracker Error:  State request out of range; returning empty state.\n");
		return r;
	}

	r.resize(BELUGA_STATE_SIZE);
	r[BELUGA_STATE_X] = m_vdTracked_X[i];
	r[BELUGA_STATE_Y] = m_vdTracked_Y[i];
	r[BELUGA_STATE_Z] = m_vdTracked_Z[i];
	r[BELUGA_STATE_HEADING] = m_vdTracked_Heading[i];
	r[BELUGA_STATE_SPEED] = m_vdTracked_Speed[i];
	r[BELUGA_STATE_ORIENTATION] = m_vdTracked_Heading[i];

	return r;
}

void BelugaTracker::setRobotData(const std::vector<double>& depth_meas,
		const std::vector<double>& speed,
		const std::vector<double>& vert,
		const std::vector<double>& turn)
{
	m_vdDepthMeasurement = depth_meas;
	m_vdSpeedCommand = speed;
	m_vdVerticalCommand = vert;
	m_vdTurnCommand = turn;
}

void BelugaTracker::getWorldXYZFromImageXYAndDepthInCamera(double* x,
		double* y,
		double* z,
		double u,
		double v,
		double d,
		bool undistort,
		unsigned int camera)
{
	if(camera >= 4 || !x || !y || !z)
	{
		return;
	}

	m_CoordinateTransforms[camera].imageAndDepthToWorld(u, v, d, x, y, z, undistort);

}

void BelugaTracker::getCameraXYFromWorldXYandDepth(int* camera, double* u, double* v, double x, double y, double depth, bool distort)
{
	if(!u || !v)
	{
		return;
	}

	if(x >= 0 && y >= 0)
	{
		*camera = 0;
	}
	if(x < 0 && y >= 0)
	{
		*camera = 1;
	}
	if(x <0 && y < 0)
	{
		*camera = 2;
	}
	if(x >= 0 && y < 0)
	{
		*camera = 3;
	}

	m_CoordinateTransforms[*camera].worldToImage(x, y, m_dWaterDepth-depth, u, v, distort);
}

/* Drawing function - gets called by the GUI
 * All of the drawing is done with OpenGL */
void BelugaTracker::doGLDrawing(int flags)
{
	unsigned int Q = flags;

	switch(flags)
	{
	case 1:
		// Q2 Drawing
		break;
	case 2:
		// Q3 Drawing
		break;
	case 3:
		// Q4 Drawing
		break;
	default:
		Q = 0;
		// Q1 Drawing
		break;
	}

    /* MT_R3 is a 3-vector class, used here for convenience */
    MT_R3 blobcenter;

    /* m_bShowBlobs is a boolean in the "Drawing Options" data group.
       If the user selects "Drawing Options" from the "Tracking" menu,
       a dialog will pop up with a check box labeled "Show blob arrows",
       and its state will be linked (via a pointer) to the value of this
       variable */
    if(m_bShowBlobs)
    {
        for (unsigned int i = 0; i < m_vBlobs[Q].size(); i++)
        {

            /* note that we have to subtract y from the frame_height
               to account for the window coordinates originating from the
               bottom left but the image coordinates originating from
               the top left */
            blobcenter.setx(m_vBlobs[Q].at(i).COMx);
            blobcenter.sety(VFLIP m_vBlobs[Q].at(i).COMy);
            blobcenter.setz( 0 );

            /* draws an arrow using OpenGL */
            MT_DrawArrow(blobcenter,  // center of the base of the arrow
                         20.0,        // arrow length (pixels)
                         MT_DEG2RAD*m_vBlobs[Q].at(i).orientation, // arrow angle
                         MT_White,//MT_Primaries[(i+1) % MT_NPrimaries], // color
                         1.0 // fixes the arrow width
                );    
        }
    }

    /* essentially the same as above, but with the tracked positions
       instead of the blob positions */
    if(m_bShowTracking)
    {
        for (unsigned int i = 0; i < m_vdTracked_X.size(); i++)
        {

            blobcenter.setx(m_vdaTracked_XC[Q][i]);
            blobcenter.sety(VFLIP m_vdaTracked_YC[Q][i]);
            blobcenter.setz( 0 );

            MT_DrawArrow(blobcenter,
                         25.0, 
                         m_vdTracked_Heading[i],
                         MT_Red,//MT_Primaries[(i+1) % MT_NPrimaries],
                         1.0 
                );    
        }
    }

	for(int i = 0; i < m_iNObj; i++)
	{
		for(unsigned int j = 0; j < m_vvdMeas_X[i].size(); j++)
		{
			if(m_vviMeas_Cam[i][j] == Q)
			{
				blobcenter.setx(m_vvdMeas_X[i][j]);
				blobcenter.sety(VFLIP m_vvdMeas_Y[i][j]);
				blobcenter.setz(0);

				MT_DrawArrow(blobcenter,
					10.0,
					MT_DEG2RAD*m_vvdMeas_Hdg[i][j],
					MT_Green,//MT_Primaries[(i+1) % MT_NPrimaries],
					0.8f);
			}
		}
	}

	for(unsigned int i = 0; i < m_SearchArea[Q].size(); i++)
	{
		MT_DrawRectangle(m_SearchArea[Q][i].x,
     		// if VFLIP is m_iFrameHeight -, then VFLIP m_iFrameHeight = 0
			VFLIP m_iFrameHeight ? m_SearchArea[Q][i].y : 
			   VFLIP m_SearchArea[Q][i].y - m_SearchArea[Q][i].height,
			m_SearchArea[Q][i].width, 
			m_SearchArea[Q][i].height);
	}

	//

}

