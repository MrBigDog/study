#pragma once
//#include "Export.h"

// 编码后的长度一般比原文多占1/3的存储空间，请保证base64code有足够的空间
/*SAXREADER_EXPORT*/ int Base64Encode(char * base64code, const char * src, int src_len = 0);
/*SAXREADER_EXPORT*/ int Base64Decode(char * buf, const char * base64code, int src_len = 0);