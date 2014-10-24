/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_UTILS_MD5_H
#define  MRPT_UTILS_MD5_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** Computes the md5 of a block of data. */
		std::string BASE_IMPEXP md5(const std::string &str);
		/** Computes the md5 of a block of data. */
		std::string BASE_IMPEXP md5(const mrpt::vector_byte &str);
		/** Computes the md5 of a block of data. */
		std::string BASE_IMPEXP md5(const unsigned char * data, const size_t len);
	}
}

#endif
