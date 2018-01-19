#pragma once

#include "export.h"

class GMLFEATURE_EXPORT guiProgressWindow
{
public:
	guiProgressWindow(void);
	virtual ~guiProgressWindow(void);
//	virtual	bool  CheckCancelButton(){;};
	virtual void  Create(){;};
	virtual void SetStatus(char*){;};
    virtual void SetRange(int nLower,int nUpper){;};
//   virtual  int  SetStep(int nStep){;};
  virtual  void  SetPos(int nPos){mPos = nPos;};
   virtual  int  GetPos(){return mPos;};
 //   virtual int  OffsetPos(int nPos){;};
    virtual void  ShowWindow(){;};
int mPos;
};

