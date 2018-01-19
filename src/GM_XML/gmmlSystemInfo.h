#ifndef gmmlSystemInfo_h__
#define gmmlSystemInfo_h__

#include "export.h"

class GEOXML_EXPORT gmmlSystemInfo
{
public:
	gmmlSystemInfo(void);
	~gmmlSystemInfo(void);
};

class GEOXML_EXPORT gmmlProgress
{
public:
	gmmlProgress(void);
	~gmmlProgress(void);

	virtual void StartProgress(int  progressmax) = 0;
	virtual void ShowText(const char* textStr) = 0;
	virtual void StopProgress() = 0;
	virtual void Progressing() = 0;
	void InitProgress();
	bool            isEnable();
protected:
	int   progressPos;
	int  progressMax;
	bool bEnable;
};


#endif // gmmlSystemInfo_h__
