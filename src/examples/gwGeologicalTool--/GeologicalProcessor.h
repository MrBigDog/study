#ifndef GEOLOGICALPROCESSOR_H
#define GEOLOGICALPROCESSOR_H

#include <gwGeologicalUtil/Config.h>
#include <string>
#include <vector>

typedef std::vector<std::string> StringVector;

class GeologicalProcessor
{
public:
	GeologicalProcessor();
	~GeologicalProcessor() {}

	void traverse(const std::string& path, StringVector& fileNameList);

	void setCurrentNode(const std::string& nodePath);
	osg::Node* getCurrentNode();

	void setUid(const std::string& uid);
	void setColor(const osg::Vec4& color);
	void setAttribuate(const std::string& attr);
	void setDiffuseMap(const std::string& diffuseMapName);

	void split(double sizeFactor);

	void publish(const std::string& path);

private:
	gwUtil::Config _config;
	osg::ref_ptr<osg::Node> _currentNode;
};



#endif // GeologicalProcess_h__
