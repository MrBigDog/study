#pragma once

#include "export.h"
//#ifndef _BASE64_INCLUDE__H__
//#define _BASE64_INCLUDE__H__

// 编码后的长度一般比原文多占1/3的存储空间，请保证base64code有足够的空间
GMXMLFILEIO_EXPORT int Base64Encode(char * base64code, const char * src, int src_len = 0); 
GMXMLFILEIO_EXPORT int Base64Decode(char * buf, const char * base64code, int src_len = 0);
