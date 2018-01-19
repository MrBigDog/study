#pragma once

#include "export.h"

#include<string>

#include "gmlAttribute.h"
class grRenderObject;
class GMObject;

class vtkProp;
class vtkProperty;

class gmlAppearance;

class GMLFEATURE_EXPORT gmlFeature
{
public:
	gmlFeature(void);
	~gmlFeature(void);

	virtual char* GetClassName() = 0;
	virtual std::string getXmlNodeName()  const {return "";};
	std::string GetFeatureName();
	void SetFeatureName(std::string name);

	std::string GetIOPath();
	void SetIOPath(std::string name);

	std::string GetFeatureID();
	void SetFeatureID(std::string);

	GMObject*          GetGeometry();
	void               SetGeometry(GMObject*);

	gmlAttributeArray *GetAttributes();
	void               SetAttributes(gmlAttributeArray*);
    void               SetGroupAttribute(std::string name,std::string value,std::string type="String");

	gmlAttributeArray *GetGeoAttributes();
	void               SetGeoAttributes(gmlAttributeArray*);
	void               SetGeoAttribute(std::string name,std::string value);
	
    virtual void  SetColletionFeature(gmlFeature*);
	virtual gmlFeature*   GetColletionFeature();

   
	gmlAppearance *GetAppearance();
	void               SetAppearance(gmlAppearance*);
	
	vtkProp*          GetActor();
	void               SetActor(vtkProp*);
protected:
	std::string         mFeatureID;
	std::string         mFeatureName;
	
	gmlAttributeArray  *mUsedAttribute;  // group Attribute;
	gmlAttributeArray  *mGeoAttribute;  // ЙЬга Attribute; 10.27

	gmlAppearance      *mAppearance;

    vtkProp*            mActor; // move to mappedfeature
	vtkProp*            mVolumeActor;  // move to mappedfeature

	std::string         mIOPath;      // file or DB
	gmlFeature*         mColletionFeature;
	// to be deleted
	//grRenderObject     *mRenderer;
	void*               mUIPointer;   // for UI  map


    GMObject           *mGeometry;
};

