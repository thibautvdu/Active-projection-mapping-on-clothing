/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CSTDOUTSTREAM_H
#define  CSTDOUTSTREAM_H

#include <mrpt/utils/CStream.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace utils
{
	/** This CStdOutStream derived class allow printing to standard out, normally
	 *    the console text output. Please notice CStdOutStream's are binary streams,
	 *    so "char *" data types only should be used if textual outputs are
	 *    desired.
	 *
	 * \sa CStream
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CStdOutStream : public CStream
	{
	protected:
		 /** Method responsible for reading from the stream:
		  *   In this class it has no effect.
		  */
		size_t  Read(void *Buffer, size_t Count) {
			MRPT_UNUSED_PARAM(Buffer); MRPT_UNUSED_PARAM(Count);
			THROW_EXCEPTION("Read-only stream");
		}

		/** Method responsible for writing to the stream.
		 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
		 */
		size_t Write(const void *Buffer,size_t Count);

	public:
		 /** Constructor
		  */
		CStdOutStream() { }

		 /** Destructor
		 */
		virtual ~CStdOutStream() { }

		/** It has no efect in this class.
		 */
		uint64_t Seek(uint64_t Offset, CStdOutStream::TSeekOrigin Origin = sFromBeginning) {
			MRPT_UNUSED_PARAM(Offset); MRPT_UNUSED_PARAM(Origin);
			THROW_EXCEPTION("Invalid operation for this kind of stream");
		}

		/** It has no efect in this class.
		 */
		uint64_t getTotalBytesCount()
			{ THROW_EXCEPTION("Invalid operation for this kind of stream"); }

		/** It has no efect in this class.
		 */
		uint64_t getPosition()
			{ THROW_EXCEPTION("Invalid operation for this kind of stream"); }

	}; // End of class def.

} // End of namespace
} // End of namespace

#endif
