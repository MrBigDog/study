#ifndef GWGEOLOGICALTUIL_FILEUTILS_H
#define GWGEOLOGICALTUIL_FILEUTILS_H

#include "Common.h"
#include "DateTime.h"
#include <vector>
#include <string>

namespace gwUtil
{
	/**
	 * Converts a filename to an absolute path.
	 */
	extern GWGEOLOGICALUTIL_EXPORT std::string getAbsolutePath(const std::string& input);

	/**
	 * Determines whether a given filename is a relative path
	 */
	extern GWGEOLOGICALUTIL_EXPORT bool isRelativePath(const std::string& fileName);

	/**
	 * Gets the full path of a file relative to a file
	 * For example, getFullPath("c:\images\vacation.jpg", "..") would return "c:\images".
	 * If relativePath is an absolute path, it is returned
	 *
	 * @param relativeTo
	 *       The name of the file to make the path relative to
	 * @param relativePath
	 *       The path relative to relativeTo
	 * @returns
	 *       The full path
	 */
	extern GWGEOLOGICALUTIL_EXPORT std::string getFullPath(
		const std::string& relativeTo,
		const std::string& relativePath);

	/**
	 * whether the given path ends with an Archive.
	 */
	extern GWGEOLOGICALUTIL_EXPORT bool isArchive(const std::string& path);

	/**
	 * whether the given path refers to a file contained inside an
	 * archive.
	 */
	extern GWGEOLOGICALUTIL_EXPORT bool isPathToArchivedFile(const std::string& path);

	/**
	 * @deprecated Please use isPathToArchivedFile instead
	 * Gets whether or not the given path contains a zip file within the path
	 */
	extern GWGEOLOGICALUTIL_EXPORT bool isZipPath(const std::string& path);

	/**
	 * Gets the path to the temp directory
	 */
	extern GWGEOLOGICALUTIL_EXPORT std::string getTempPath();

	/**
	 * Sets the "Last Modified" timestamp of a file to Now.
	 */
	extern GWGEOLOGICALUTIL_EXPORT bool touchFile(const std::string& path);

	/**
	 * Gets the "Last Modified" timestamp of a file.
	 */
	extern GWGEOLOGICALUTIL_EXPORT TimeStamp getLastModifiedTime(const std::string& path);

	/**
	 * Gets a temporary filename
	 * @param prefix
	 *        The prefix of the temporary filename
	 * @param suffix
	 *        The suffix of the temporary filename
	 */
	extern GWGEOLOGICALUTIL_EXPORT std::string getTempName(const std::string& prefix = "", const std::string& suffix = "");

	/** Make a new directory.  Returns true if directory exists or was created.
	 * Note:  This is adapted from osg's makeDirectory function to ensure that it works with concurrent create attempts.
	*/
	extern GWGEOLOGICALUTIL_EXPORT bool makeDirectory(const std::string &directoryPath);

	/** Make a new directory for a given file.
	 * Note:  This is adapted from osg's makeDirectoryForFile to call our custom makeDirectory function.
	*/
	extern GWGEOLOGICALUTIL_EXPORT bool makeDirectoryForFile(const std::string &filePath);

	/**
	 * Utility class that processes files and directories recursively.
	 */
	class GWGEOLOGICALUTIL_EXPORT DirectoryVisitor
	{
	public:
		/**
		 * Create a new DirectoryVisitor
		 */
		DirectoryVisitor();

		/**
		 * Processes a file.  Override this in your subclass.
		 */
		virtual void handleFile(const std::string& filename);

		/**
		 * Processes a directory.
		 * @returns
		 *     true if the directory should be traversed, false otherwise.
		 */
		virtual bool handleDir(const std::string& path);

		/**
		 * Traverse into a directory
		 * @param path
		 *     The path to traverse
		 */
		virtual void traverse(const std::string& path);
	};

	/**
	 * Utility class that recursively collects all files within a directory.
	 */
	class GWGEOLOGICALUTIL_EXPORT CollectFilesVisitor : public DirectoryVisitor
	{
	public:
		CollectFilesVisitor();

		virtual void handleFile(const std::string& filename);

		/**
		 * The collected filenames.
		 */
		std::vector< std::string > filenames;
	};
}

#endif
