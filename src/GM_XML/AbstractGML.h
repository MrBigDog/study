#ifndef AbstractGML_h__
#define AbstractGML_h__

#include "export.h"
#include <string>
#include <ostream>
#include <vector>

// AbstractDataComppnent
struct AttributeValue
{
	std::string fieldType;
	std::string fieldName;
	std::string fieldValue; // id
};

struct AttributeField
{
	std::string fieldName;
	std::string fieldType;
	std::string fieldDescription;
};

namespace gml
{

	class GEOXML_EXPORT AbstractGML
	{
	public:
		AbstractGML(void);
		~AbstractGML(void);
		virtual std::string GetClassName() { return "AbstractGML"; };
	public:
		/// @name 本对象的全局唯一ID。
		//@{
		std::string getID() const;
		void setID(std::string  id);
		//@}

		long getDBID();
		void setDBID(long id);
		//@}

		/// @name 名字
		//@{
		std::string getName() const;
		void setName(std::string  name);
		//@}

		/// @name 本对象的文件路径
		//@{
		std::string getIOPath() const;
		void setIOPath(std::string  value);
		//@}

		AbstractGML* getContainer() const;
		void setContainer(const AbstractGML* c);
		virtual void deleteMembers() {};
	private:
		AbstractGML *container_;
		/// 本对象的全局唯一ID。
		std::string id_;
		/// 名字
		std::string name_;
		/// 文件路径
		std::string IOPath;

		// 数据库DB ID
		long db_id;
	};

}
#endif // AbstractGML_h__