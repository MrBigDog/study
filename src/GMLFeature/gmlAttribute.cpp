#include "gmlAttribute.h"


gmlAttribute::gmlAttribute(void)
{
	 mValueType = UNKNOWN;
}


gmlAttribute::~gmlAttribute(void)
{
}

gmlAttribute::gmlAttribute( const gmlAttribute & rgb)
{
	this->mFieldName =  rgb.GetName();
	this->mValue = rgb.GetValue();
	this->mValueType = rgb.GetValueType();
	this->mAlias = rgb.GetAlias();
}

 gmlAttribute::gmlAttribute( const gmlAttribute * rgb)
{
	this->mFieldName =  rgb->GetName();
	this->mValue = rgb->GetValue();
	this->mValueType = rgb->GetValueType();
	this->mAlias = rgb->GetAlias();
}


void gmlAttribute::SetValueTypeString(std::string n)
{
	if(n == "String")
		mValueType = STRING;

	if(n == "Text")
		mValueType = STRING;

	if(n == "Long")
		mValueType = LONG;

	if(n == "Count")
		mValueType = LONG;

	if(n == "Double")
		mValueType = DOUBLE;

	if(n == "Float")
		mValueType = FLOAT;  
}
 
gmlAttribute& gmlAttribute::operator =(const gmlAttribute& rgb)
{
	this->mFieldName = rgb.GetName();
	mValue = rgb.GetValue();
	mValueType = rgb.GetValueType();
	mAlias = rgb.GetAlias();

	return *this;
}