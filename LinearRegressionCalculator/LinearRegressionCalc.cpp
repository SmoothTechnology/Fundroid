#include <iostream>
#include <math.h>

using namespace std;

//#define REAL float
#define REAL double
#define MAX_READS 300

inline static REAL sqr(REAL x) {
    return x*x;
}


int linreg(int n, REAL x[], REAL y[], REAL* m, REAL* b, REAL* r)
{
    REAL   sumx = 0.0;                        /* sum of x                      */
    REAL   sumx2 = 0.0;                       /* sum of x**2                   */
    REAL   sumxy = 0.0;                       /* sum of x * y                  */
    REAL   sumy = 0.0;                        /* sum of y                      */
    REAL   sumy2 = 0.0;                       /* sum of y**2                   */

   for (int i=0;i<n;i++)   
      { 
      sumx  += x[i];       
      sumx2 += sqr(x[i]);  
      sumxy += x[i] * y[i];
      sumy  += y[i];      
      sumy2 += sqr(y[i]); 
      } 

   REAL denom = (n * sumx2 - sqr(sumx));
   if (denom == 0) {
       // singular matrix. can't solve the problem.
       *m = 0;
       *b = 0;
       if (r) *r = 0;
       return 1;
   }

   *m = (n * sumxy  -  sumx * sumy) / denom;
   *b = (sumy * sumx2  -  sumx * sumxy) / denom;
   if (r!=NULL) {
      *r = (sumxy - sumx * sumy / n) /          /* compute correlation coeff     */
            sqrt((sumx2 - sqr(sumx)/n) *
            (sumy2 - sqr(sumy)/n));
   }

   return 0; 
}

void PrintXY(REAL x[], REAL y[], int n)
{
	for(int i = 0; i < n; i++)
	{
		cout << "(" << x[i] << "," << y[i] << ")";
	}

	cout << endl;
}

void FindBestFitLineInDataSet(REAL x[], REAL y[], int n, int WallShouldBeOnRight)
{
	int acceptableNumberOfReads = 3;
	double acceptableRSquared = 0.95;
	int maxCycles = 5;
	REAL newX[MAX_READS];
	REAL newY[MAX_READS];

	REAL m,b,r;
    int lineStatus = linreg(n,x,y,&m,&b,&r);
    cout << "LineStatus: " <<  lineStatus << endl;

    if(lineStatus == 1)
    {
    	cout << "Perfectly Parallel" << endl;
    }
    else
    {
    	double Angle = atan(m)*180/3.14;
    	REAL rSqured = r*r;

    	cout << "Slope: " << m <<  " SlopeAngle: " << Angle << " Intercept: " << b << " R^2:" << rSqured << endl;

    	int curCycle = 0;
    	while(rSqured < acceptableRSquared && curCycle < maxCycles)
    	{
    		PrintXY(x, y, n);
    		curCycle++;
    		// Cut off all data on left side of line
    		if(WallShouldBeOnRight)
    		{
    			int numAdded = 0;
    			for(int i = 0; i < n; i++)
    			{
    				double expectedXforY = (y[i]-b)/m;
    				if(expectedXforY <= x[i])
    				{
    					newX[numAdded] = x[i];
    					newY[numAdded] = y[i];
    					numAdded++;
    				}
    				else
    				{
    					cout << "Expected X:" << expectedXforY << " For ValY: " << y[i] << endl;
    				}
    			}

    			n = numAdded;
    			for(int i = 0; i < n; i++)
    			{
    				x[i] = newX[i];
    				y[i] = newY[i];
    			}

    			lineStatus = linreg(n,x,y,&m,&b,&r);
    			rSqured = r*r;
    			cout << "LineStatus: " <<  lineStatus << endl;
    			cout << "Slope: " << m <<  " SlopeAngle: " << Angle << " Intercept: " << b << " R^2:" << rSqured << endl;
    		}
    	}
    }
}

int main()
{
	int curN = 5;
	int WallShouldBeOnRight = 1;

    REAL x[6]= {3, 9, 15, 21,  10};
    REAL y[6]= {20, 16, 12, 8, 4};

    FindBestFitLineInDataSet(x, y, curN, WallShouldBeOnRight);

    return 0;
}