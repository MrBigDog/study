#ifndef FeatureStyle_h__
#define FeatureStyle_h__



#include "export.h"

#include <vector>
#include <string>
//#include "StyleTable.h"
namespace se
{
	class GeoSymbolizer;
	class GEOXML_EXPORT FeatureStyleRule
	{
	public:
		FeatureStyleRule();
		~FeatureStyleRule();
		std::string FilterType;   // ogc:PropertyIsEqualTo
		std::string PropertyName; // ogc:PropertyName
		std::string Literal;      // ogc:Literal
		std::string SymbolizerType;	//记录Point, Line, Surface三种不同的Symbolizer类型

		void SetSymbolizer(GeoSymbolizer * s);
		GeoSymbolizer *GetSymbolizer();

	protected:
		GeoSymbolizer *Symbolizer;
	};

	class GEOXML_EXPORT FeatureType3DStyle
	{
	public:
		FeatureType3DStyle(void);
		~FeatureType3DStyle(void);

		//@{
		std::string getFeatureTypeName() const;
		void setFeatureTypeName(const std::string& name);
		//@}

		int getRuleCount();

		void addRule(FeatureStyleRule* rule);
		FeatureStyleRule* getRule(int i);
		void deleteAllrules();

	public:
		void write(std::ostream& xmlStream) const;

	protected:
		virtual std::string getXmlNodeName() const;
		virtual void writeExt(std::ostream& xmlStream) const;

	private:
		std::string featureTypeName_;
		std::vector<FeatureStyleRule*> Rules;
	};

}

#endif // FeatureStyle_h__