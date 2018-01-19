#ifndef GWGEOLOGICALUTIL_XML_UTILS_H
#define GWGEOLOGICALUTIL_XML_UTILS_H

#include "Common.h"
#include "Config.h"
#include "StringUtils.h"
#include "URI.h"
#include <osg/Referenced>
#include <osg/ref_ptr>
#include <string>
#include <vector>
#include <map>
#include <stack>

// XML support utilites.

namespace gwUtil
{
	class GWGEOLOGICALUTIL_EXPORT XmlNode : public osg::Referenced
	{
	public:
		XmlNode();

		virtual ~XmlNode() { }

		virtual bool isElement() const = 0;

		virtual bool isText() const = 0;
	};

	typedef std::vector<osg::ref_ptr<XmlNode> > XmlNodeList;

	typedef std::map<std::string, std::string> XmlAttributes;

	class GWGEOLOGICALUTIL_EXPORT XmlElement : public XmlNode
	{
	public:
		XmlElement(const std::string& name);

		XmlElement(const std::string& name, const XmlAttributes& attrs);

		XmlElement(const Config& conf);

		virtual ~XmlElement() { }

		const std::string& getName() const;

		void setName(const std::string& name);

		XmlAttributes& getAttrs();

		const XmlAttributes& getAttrs() const;

		const std::string& getAttr(const std::string& key) const;

		XmlNodeList& getChildren();

		const XmlNodeList& getChildren() const;

		XmlElement* getSubElement(const std::string& name) const;

		XmlNodeList getSubElements(const std::string& name) const;

		/**
		 * Finds the first element matching the name. This will match the
		 * current element (this), or the first matching element in this
		 * nodes subhierarchy.
		 */
		const XmlElement* findElement(const std::string& name) const;

		std::string getText() const;

		std::string getSubElementText(const std::string& tag) const;

		void addSubElement(const std::string& tag, const std::string& text);

		void addSubElement(const std::string& tag, const Properties& attrs, const std::string& text);

		virtual Config getConfig(const std::string& sourceURI) const;

	public: // XmlNode
		virtual bool isElement() const { return true; }

		virtual bool isText() const { return false; }

		virtual bool isInclude() const { return toLower(name) == "xi:include"; }

	private:
		std::string name;
		XmlAttributes attrs;
		XmlNodeList children;
	};

	typedef std::vector<osg::ref_ptr<XmlElement> > XmlElementList;

	typedef std::stack<osg::ref_ptr<XmlElement> > XmlElementStack;

	typedef std::stack<XmlElement*> XmlElementNoRefStack;

	class GWGEOLOGICALUTIL_EXPORT XmlText : public XmlNode
	{
	public:
		XmlText(const std::string& value);

		virtual ~XmlText() { }

		const std::string& getValue() const;

	public: // XmlNode
		virtual bool isElement() const { return false; }

		virtual bool isText() const { return true; }

	private:
		std::string value;
	};

	class GWGEOLOGICALUTIL_EXPORT XmlDocument : public XmlElement
	{
	public:
		XmlDocument();

		XmlDocument(const Config& conf);

		virtual ~XmlDocument();

		static XmlDocument* load(const std::string& location, const osgDB::Options* dbOptions = 0L);

		static XmlDocument* load(const URI& uri, const osgDB::Options* dbOptions = 0L);

		static XmlDocument* load(std::istream& in, const URIContext& context = URIContext());

		void store(std::ostream& out) const;

		const std::string& getName() const;

		virtual Config getConfig() const;

	protected:
		URI                      _sourceURI;
		std::string              _name;
		osg::ref_ptr<XmlElement> _root;
	};
}

#endif // OSGEARTH_XML_UTILS_H
