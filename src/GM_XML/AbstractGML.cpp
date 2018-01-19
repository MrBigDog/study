#include "AbstractGML.h"
#include "gmmlSystem.h"
using namespace gml;

AbstractGML::AbstractGML(void)
{
	container_ = NULL;
	 db_id = -1;

}

AbstractGML::~AbstractGML(void)
{
	deleteMembers();
} 

AbstractGML* AbstractGML::getContainer() const
{
	return container_;
}

void AbstractGML::setContainer(const AbstractGML* c)
{
	container_ = (AbstractGML*)c;
}

 

void AbstractGML::setDBID(long id)
{
	  db_id = id;
}

long AbstractGML::getDBID() 
{
#ifdef _DEBUG
	if(db_id == -1)
    {
		std::string str = this->GetClassName() + "DBID == -1";
		gmmlDisplayErrorandWarningText(str.c_str());
	}
#endif

	return db_id;


}

void AbstractGML::setID( std::string  id)
{
	id_ = id;
	if (name_.empty())
		name_ = id_;
}

std::string AbstractGML::getID() const
{
	return id_;
}

std::string AbstractGML::getName() const
{
	return name_;
}

void AbstractGML::setName( std::string  name)
{
	name_ = name;
}

std::string AbstractGML::getIOPath() const
{
	return IOPath;
}
void AbstractGML::setIOPath( std::string  value)
{
	IOPath = value;
}

