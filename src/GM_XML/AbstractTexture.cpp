#include "AbstractTexture.h"

using namespace gmml;
AbstractTexture::AbstractTexture(void)
{
	ImageURI="";
	MimeType="";
	WrapMode = 0;
    RepeatedCount = 1;
	 FaceNorm = 0;//(0,0,1)
	  BorderColor[0]= BorderColor[1]= BorderColor[2]= BorderColor[3]=1;
	    ReferencePoint[0]= ReferencePoint[1]=ReferencePoint[2]=1;
		   Orientation[0]=  Orientation[1]=  Orientation[2]=  Orientation[3]=1;
}


AbstractTexture::~AbstractTexture(void)
{
}
