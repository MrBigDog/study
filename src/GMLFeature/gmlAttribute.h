#pragma once

#include "export.h"

#include<string>
#include<vector>

class GMLFEATURE_EXPORT gmlAttribute
{
public:
	gmlAttribute(void);
	~gmlAttribute(void);
	 
	gmlAttribute(const gmlAttribute& t);	
	 gmlAttribute(const  gmlAttribute* t);	
 	
    gmlAttribute(std::string name);		 
    gmlAttribute& operator=(const  gmlAttribute& attr);  // ¸³Öµ

	enum gmlValueType{
	FLOAT,
	DOUBLE,
	BOOL,
	SHORT,
	LONG,
	DATE,
    STRING,
	UNKNOWN};


	void SetName(std::string n){  mFieldName = n;};
	void SetAlias(std::string n){  mAlias = n;};
	void SetValue(std::string n){  mValue = n;};
	void SetValueType(gmlValueType n){  mValueType = n;};
	void SetValueTypeString(std::string n);
	
	std::string GetName() const {return mFieldName;};
	std::string GetAlias()const {return mAlias;};
	gmlAttribute::gmlValueType GetValueType() const {return mValueType;};
	std::string GetValue() const{return mValue;};
   
public:
	std::string    mFieldName;
	std::string    mValue;
    gmlValueType   mValueType;     
	std::string    mAlias;  
};

typedef std::vector<gmlAttribute> gmlAttributeArray;

