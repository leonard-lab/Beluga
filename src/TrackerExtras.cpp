#include "TrackerExtras.h"

#include "MT/MT_Core/support/mathsupport.h"


/* helper function.  Basic FIFO buffer with N_hist entries
 *
 * Given new values for x and y, do
 * if size of X < N_hist,
 *    X = [existing values.... x]
 * else (have N_hist values)
 *    drop the first value, shift all of the other values forward
 *     (i.e. X[k] = X[k+1]), and set the last value to x
 *
 * The same is done for Y. */
void rollHistories(std::vector<double>* X,
                   std::vector<double>* Y,
                   double x,
                   double y,
                   unsigned int N_hist)
{
    if(X->size() < N_hist)
    {
        X->push_back(x);
        Y->push_back(y);
    }
    else
    {
        for(unsigned int i = 0; i < X->size()-1; i++)
        {
            X->at(i) = X->at(i+1);
            Y->at(i) = Y->at(i+1);
        }
        X->at(X->size()-1) = x;
        Y->at(Y->size()-1) = y;
    }
}

/* helper function.  Returns false if any element of M is either NaN
 * or larger in magnitude than max_val */
bool CvMatIsOk(const CvMat* M, double max_val)
{
    double v;
    for(int i = 0; i < M->rows; i++)
    {
        for(int j = 0; j < M->cols; j++)
        {
            v = cvGetReal2D(M, i, j);
            if(MT_isnan(v) || fabs(v) > max_val)
            {
                return false;
            }
        }
    }
    return true;
}

CvRect searchRectAtPointWithSize(double x_center, double y_center, double size)
{
	return cvRect((int)(x_center-0.5*size), (int)(y_center-0.5*size), (int) size, (int)size);
}

bool cvRectsIntersect(const CvRect& a, const CvRect& b)
{
	return a.x < (b.x+b.width) && (a.x+a.width) > b.x &&
		a.y < (b.y+b.height) && (a.y+a.height) > b.y;
}

bool pointInCvRect(double px, double py, const CvRect& r)
{
	return (px >= r.x) && (px <= (r.x+r.width)) 
		&& (py >= r.y) && (py <= (r.y+r.height));
}

CvRect unionOfCvRects(const CvRect& a, const CvRect&b)
{
	int xmin = MT_MIN(a.x, b.x);
	int xmax = MT_MAX(a.x+a.width, b.x+b.width);
	int ymin = MT_MIN(a.y, b.y);
	int ymax = MT_MAX(a.y+a.height, b.y+b.height);

	return cvRect(xmin, ymin, xmax-xmin, ymax-ymin);
}

std::vector<unsigned int> unionOfIndexSets(const std::vector<unsigned int>& a,
										   const std::vector<unsigned int>& b)
{
	std::vector<unsigned int> result;
	result.reserve(a.size()+b.size());
	result.insert(result.end(), a.begin(), a.end());
	result.insert(result.end(), b.begin(), b.end());
	return result;
}

void combineSearchAreas(std::vector<CvRect>* searchAreas, 
						std::vector<std::vector<unsigned int> >* searchIndexes)
{
	if(searchAreas->size() <= 1)
	{
		return;
	}

	std::vector<CvRect> inAreas = *searchAreas;
	std::vector<std::vector<unsigned int> > inIndexes = *searchIndexes;

	std::vector<CvRect> outAreas;
	std::vector<std::vector<unsigned int> > outIndexes;
	int num_joined = 0;

	do
	{
		outAreas.resize(0);
		outIndexes.resize(0);
		num_joined = 0;

		for(unsigned int i = 0; i < inAreas.size(); i++)
		{
			for(unsigned int j = i+1; j < inAreas.size(); j++)
			{
				if(cvRectsIntersect(inAreas[i], inAreas[j]))
				{
					num_joined++;
					outAreas.push_back(unionOfCvRects(inAreas[i], inAreas[j]));
					outIndexes.push_back(unionOfIndexSets(inIndexes[i], inIndexes[j]));
				}
				else
				{
					outAreas.push_back(inAreas[i]);
					outIndexes.push_back(inIndexes[i]);
				}
			}
		}

		inAreas = outAreas;
		inIndexes = outIndexes;
	} while(num_joined > 0);

	*searchAreas = inAreas;
	*searchIndexes = inIndexes;

}
