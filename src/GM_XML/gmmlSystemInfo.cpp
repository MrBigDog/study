#include "gmmlSystemInfo.h"
#include "gmmlSystem.h"

gmmlSystemInfo::gmmlSystemInfo(void)
{
}


gmmlSystemInfo::~gmmlSystemInfo(void)
{
}


//

gmmlProgress* g_Progress=0;
 
gmmlProgress::gmmlProgress(void)
{
	g_Progress = 0;
	InitProgress();
}


gmmlProgress::~gmmlProgress(void)
{
}

bool           gmmlProgress:: isEnable()
{
	return bEnable;
}

void gmmlProgress::InitProgress()
{
	bEnable = 0;
	progressPos = 0;
	progressMax = 0;
}
 
 

void gmmlStartProgress(int  progressmax)
{
	if(g_Progress && !g_Progress->isEnable())
		g_Progress->StartProgress(  progressmax);
}
void gmmlShowProgress(const char* textstr)
{
	if(g_Progress)
	{
		g_Progress->ShowText(textstr);
	}
}
void gmmlProgressing()
{
	if(g_Progress)
	{
		g_Progress->Progressing();
	}
}
void gmmlStopProgress()
{
	if(g_Progress)
		g_Progress->StopProgress();

	if(g_Progress)
		g_Progress->InitProgress();
}
