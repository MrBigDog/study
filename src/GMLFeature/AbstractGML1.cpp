#include "AbstractGML1.h"

 

AbstractGML::AbstractGML(void)
{
}

AbstractGML::~AbstractGML(void)
{
}

std::string AbstractGML::getID() const
{
	return id_;
}

void AbstractGML::setID(const std::string& id)
{
	id_ = id;
	if (name_.empty())
		name_ = id_;
}

std::string AbstractGML::getName() const
{
	return name_;
}

void AbstractGML::setName(const std::string& name)
{
	name_ = name;
}

void AbstractGML::write(std::ostream& xmlStream) const
{
	using namespace std;
	string xmlNodeName = getXmlNodeName();
	xmlStream << "<" << xmlNodeName << " gml:id=\"" << getID() << "\">" << endl;
	xmlStream << "<gml:name>" << getName() << "</gml:name>" << endl;
	writeExt(xmlStream);
	xmlStream << "</" << xmlNodeName << ">" << endl;
}