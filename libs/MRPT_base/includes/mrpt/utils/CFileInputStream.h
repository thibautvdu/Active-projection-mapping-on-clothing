/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CFileInputStream_H
#define  CFileInputStream_H

#include <mrpt/utils/CStream.h>
#include <fstream>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace utils
	{
		/** This CStream derived class allow using a file as a read-only, binary stream.
		 *
		 * \sa CStream, CFileStream, CFileGZInputStream
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CFileInputStream : public CStream, public CUncopiable
		{
		protected:
			 /** Method responsible for reading from the stream.
			 */
			size_t  Read(void *Buffer, size_t Count);

			/** Method responsible for writing to the stream.
			 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
			 */
			size_t  Write(const void *Buffer, size_t Count);

		private:
			std::ifstream 	m_if;		//!< The actual input file stream.

			// DECLARE_UNCOPIABLE( CFileInputStream )

		public:
			 /** Constructor
			  * \param fileName The file to be open in this stream
			  * \exception std::exception On error trying to open the file.
			  */
			CFileInputStream(const std::string &fileName );

			 /** Default constructor
			  */
			CFileInputStream();

			 /** Open a file for reading
			  * \param fileName The file to be open in this stream
			  * \return true on success.
			  */
			bool open(const std::string &fileName );

			/** Close the stream. */
			void close();

			 /** Destructor
			 */
			 virtual ~CFileInputStream();

			 /** Says if file was open successfully or not.
			  */
			 bool  fileOpenCorrectly();

			 /** Will be true if EOF has been already reached.
			   */
			 bool checkEOF();

			/** Method for moving to a specified position in the streamed resource.
			 *   See documentation of CStream::Seek
			 */
			uint64_t Seek( uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning);

			/** Method for getting the total number of bytes in the buffer.
			 */
			uint64_t getTotalBytesCount();

			/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
			 */
			uint64_t getPosition();

			/** Reads one string line from the file (until a new-line character)
			  * \return true if a line has been read, false on EOF or error.
			  */
			bool readLine( std::string &str );

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
