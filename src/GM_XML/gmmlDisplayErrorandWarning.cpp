#include "gmmlDisplayErrorandWarning.h"
#include "gmmlSystem.h"
gmmlDisplayErrorandWarning* g_DisplayErrorandWarning=0;
gmmlDisplayErrorandWarning::gmmlDisplayErrorandWarning(void)
{
	g_DisplayErrorandWarning = 0;
}


gmmlDisplayErrorandWarning::~gmmlDisplayErrorandWarning(void)
{
	if(g_DisplayErrorandWarning)
	{
		delete g_DisplayErrorandWarning;
		g_DisplayErrorandWarning = 0;
	}
}


void gmmlDisplayErrorandWarningText(const char* textstr)
{
	if(g_DisplayErrorandWarning)
	g_DisplayErrorandWarning->DisplayText(textstr);
}