/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef color_maps_H
#define color_maps_H

#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
	namespace utils
	{
		/** Transform HSV color components to RGB, all of them in the range [0,1]
		  * \sa rgb2hsv
		  */
		void BASE_IMPEXP hsv2rgb(
			float	h,
			float	s,
			float	v,
			float	&r,
			float	&g,
			float	&b);

		/** Transform RGB color components to HSV, all of them in the range [0,1]
		  * \sa hsv2rgb
		  */
		void BASE_IMPEXP rgb2hsv(
			float	r,
			float	g,
			float	b,
			float	&h,
			float	&s,
			float	&v );

		/** Different colormaps
		  * \sa mrpt::vision::colormap
		  */
		enum TColormap
		{
			cmGRAYSCALE = 0,
			cmJET
		};

		/** Transform a float number in the range [0,1] into RGB components. Different colormaps are available.
		  */
		void BASE_IMPEXP colormap(
			const TColormap &color_map,
			const float	color_index,
			float	&r,
			float	&g,
			float	&b);

		/** Computes the RGB color components (range [0,1]) for the corresponding color index in the range [0,1] using the MATLAB 'jet' colormap.
		  * \sa colormap
		  */
		void BASE_IMPEXP jet2rgb(
			const float	color_index,
			float	&r,
			float	&g,
			float	&b);
	}
}


#endif
