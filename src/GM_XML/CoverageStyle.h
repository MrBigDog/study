#ifndef CoverageStyle_h__
#define CoverageStyle_h__

#include "export.h"
#include <vector>
#include <string>
//#include "StyleTable.h"

namespace se
{
	class GeoSymbolizer;
	class GEOXML_EXPORT  CoverageStyleRule
	{
	public:
		CoverageStyleRule();
		~CoverageStyleRule();
		std::string FilterType;   // ogc:PropertyIsEqualTo
		std::string PropertyName; // ogc:PropertyName
		std::string Literal;      // ogc:Literal
		std::string SymbolizerType;

		void SetSymbolizer(GeoSymbolizer * s);
		GeoSymbolizer *GetSymbolizer(); // rastersymbolizer

	protected:
		GeoSymbolizer *Symbolizer; // rastersymbolizer
	};

	class GEOXML_EXPORT CoverageType3DStyle
	{
	public:
		CoverageType3DStyle(void);
		~CoverageType3DStyle(void);

		//@{
		std::string getFeatureTypeName() const;
		void setFeatureTypeName(const std::string& name);
		//@}

		int getRuleCount();

		void addRule(CoverageStyleRule* rule);
		CoverageStyleRule* getRule(int i);
		void deleteAllrules();

	public:
		void write(std::ostream& xmlStream) const;

	protected:
		virtual std::string getXmlNodeName() const;
		virtual void writeExt(std::ostream& xmlStream) const;

	private:
		std::string featureTypeName_;
		std::vector<CoverageStyleRule*> Rules;
	};

}
#endif // CoverageStyle_h__
