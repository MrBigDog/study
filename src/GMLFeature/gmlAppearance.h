#pragma once
#include "gmlfeature.h"
#include<string>
#include "vtkOpenGLProperty.h"
class vtkProperty;

class GMLFEATURE_EXPORT gmlAppearance :public vtkOpenGLProperty
{
public:
	gmlAppearance(void);
	~gmlAppearance(void);
	static gmlAppearance *New();
	virtual char* GetClassName() { return "gmlAppearance"; };

	enum AppearanceType
	{
		POINT = 0,
		LINE,
		SURFACE
	};


	struct X3DTexture
	{
		X3DTexture()
		{
			RepeatedCount = 1;
			WrapMode = 0;
			MapedNorm = 2;
		};

		std::string ImageURI;
		std::string MineType;
		int WrapMode;
		int MapedNorm;  // 面法向 1 2 前后 3 上下 
		int RepeatedCount;
		double Bound[4];
	};

	std::string GetGeoFeatureID();
	void SetGeoFeatureID(std::string);

	std::string GetPropertyName();
	void SetPropertyName(std::string);

	void SetLineTypeString(std::string t);

	AppearanceType GetAppearanceType();
	void SetAppearanceType(AppearanceType s);

	void CreateTexture();
	int UseTexure();
	X3DTexture* GetTxture();
	void SetUseTexure(int);
	void SetChangedFlag(int);
	int GetChangedFlag();
protected:
	std::string         mGeoFeatureID;  // 2o11.10.27 for sld and geosciml
	std::string         mPropertyName;  // 2o11.10.27 for sld and geosciml
	int                 mUseTexure;
	int                 mChangedFlag;
	X3DTexture          *mTexture;
	AppearanceType      mAppearanceType;
};

