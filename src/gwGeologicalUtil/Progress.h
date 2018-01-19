#ifndef GWGEOLOGICALUTIL_PROGRESS_H
#define GWGEOLOGICALUTIL_PROGRESS_H 1

#include "Common.h"
#include "Containers.h"

namespace gwUtil
{
	/**
	* ProgressCallback is a general purpose interface for functions that need to report progress.
	*/
	class GWGEOLOGICALUTIL_EXPORT ProgressCallback : public osg::Referenced
	{
	public:
		/**
		* Creates a new ProgressCallback
		*/
		ProgressCallback();
		virtual ~ProgressCallback() { }

		/**
		 * Report an error and set the canceled flag to true.
		 */
		virtual void reportError(const std::string& msg);

		/**
		* Callback function that will be called.
		* @param current
		*        The amount of work done in the current stage
		* @param total
		*        The total amount of work to be done in the current stage
		* @param stage
		*        Stage of the operation we're currently in
		* @param totalStages
		*        Total number of stages in the operation
		* @param msg
		*        Description of what is being done. Useful when total is unknown.
		* @param returns
		*        Returns true if the current task should be cancelled, false otherwise.
		*/
		virtual bool reportProgress(
			double             current,
			double             total,
			unsigned           currentStage,
			unsigned           totalStages,
			const std::string& msg);

		/**
		 * Convenience functions
		 */
		bool reportProgress(double current, double total, const std::string& msg) {
			return reportProgress(current, total, 0, 1, msg);
		}
		bool reportProgress(double current, double total) {
			return reportProgress(current, total, 0, 1, "");
		}

		/**
		 * called when the process starts
		 */
		virtual void onStarted() { }

		/**
		 * called when the process completed
		 */
		virtual void onCompleted() { }

		/**
		 * Sets the cancelation flag
		 */
		virtual void cancel() { _canceled = true; }

		/**
		 * Whether cancelation was requested
		 */
		virtual bool isCanceled() { return _canceled; }

		/**
		 * Whether reportError was called
		 */
		virtual bool failed() { return _failed; }

		/**
		 * Status/error message
		 */
		std::string& message() { return _message; }
		const std::string& message() const { return _message; }

		/**
		 * Resets the canceled flag.
		 */
		void reset() { _canceled = false; }

		/**
		*Whether or not the task should be retried.
		*/
		bool needsRetry() const { return _needsRetry; }

		/**
		 * Sets whether or not the task should be retried
		 */
		void setNeedsRetry(bool needsRetry) { _needsRetry = needsRetry; }

		/**
		 * Access user stats
		 */
		typedef fast_map<std::string, double> Stats;
		Stats& stats() { return _stats; }
		double& stats(const std::string& name);
		bool& collectStats() { return _collectStats; }
		const bool& collectStats() const { return _collectStats; }

	protected:
		std::string       _message;
		mutable  bool     _needsRetry;
		mutable  bool     _canceled;
		mutable  bool     _failed;
		mutable  Stats    _stats;
		mutable  bool     _collectStats;
	};


	/**
	* ConsoleProgressCallback is a simple ProgressCallback that reports progress to the console
	*/
	class GWGEOLOGICALUTIL_EXPORT ConsoleProgressCallback : public ProgressCallback
	{
	public:
		/**
		* Creates a new ConsoleProgressCallback
		*/
		ConsoleProgressCallback();
		virtual ~ConsoleProgressCallback() { }

		virtual void reportError(const std::string& msg);

		/**
		* Callback function that will be called.
		* @param current
		*        The amount of work done in the current stage
		* @param total
		*        The total amount of work to be done in the current stage
		* @param stage
		*        Stage of the operation we're currently in
		* @param totalStages
		*        Total number of stages in the operation
		* @param msg
		*        Description of what is being done. Useful when total is unknown.
		* @param returns
		*        Returns true if the current task should be cancelled, false otherwise.
		*/
		virtual bool reportProgress(
			double             current,
			double             total,
			unsigned           currentStage,
			unsigned           totalStages,
			const std::string& msg);
	};
}

#endif
