// libAeon
// Copyright (C) 2010 Aeon Scientific
//
// Brad Kratochvil, Michael Kummer, Sandro Erni
//
// For licensing information see the EULA.txt file included with this
// distribution.

// $Id: adebug.h 2764 2012-11-28 12:45:36Z oezcanm $

#ifndef ADEBUG_H
#define ADEBUG_H

#include <QTextStream>
#include <QMutex>
#include <ros/ros.h>

/// _ID_ can be used to output the line at which the ADebug was
/// instantiated
#define _ID_ Q_FUNC_INFO

/// @brief our debugging output class
///
/// This is the primary debug class for Daedalus and allows runtime debugging
/// of the code.  The basic idea is that we have a system wide debug level
/// which can range between 0 and 10.  If the debug level exceeds the level set
/// in the constructor, a message is output displaying whatever is on the
/// stream.  If the debug level is -1, a popup window will be created to warn
/// the user.
///
/// @code
/// AOUT(0) << "this is a debug message" << endl;
/// AOUT(0) << "you can also put variables: " << 10 << endl;
/// @endcode
/// @note This is currently *not* thread safe!
class ADebug : public QTextStream
{
  public:

    /// default constructor
    ADebug();

    /// constructor with debug le<<(vel and string to be prepended to output
    ADebug(int l, const QString &text = "");

    /// the destructor is what triggers the output
    ~ADebug();

    /// set the global debug level for the main program (or a plugin)
    static void setDebugLevel(int level);

    /// returns the debug level
    static int debugLevel();

    /// debug level for critical errors that will pop up a message box
    static int msgCritical;
    /// debug level for warnings that will pop up a message box
    static int msgWarning;
    /// debug level for warnings that will not be displayed to a normal user
    static int warn;
    /// debug level for errors that will always be displayed
    static int error;
    /// debug level for displaying all AOUT statements
    static int all;

    /// the current debug level
    static int dbgLevel;


  private:

    void openLog();
    void writeLog(const QString& val);
    void closeLog();


    static QTextStream derr;

    static QMutex lock;

    static const char* logName;

    static bool syslogOpen;

    static bool writeSysLog;
    static bool writeStdErr;

    QString str;
    QString id;

    int level;
};

/// @note document this similar to
//http://www.orxonox.net/browser/code/trunk/src/libraries/util/Debug.h
//91	@brief
//92	    Logs text output: use exactly like std::cout, but specify an output
//93	    level as argument.
//94	@details
//95	    (a > b ? 0 : c << "text") is equivalent to (a > b ? 0 : (c << "text"))
//96	    where (a > b ? 0 : ) stands for COUT(x). This should explain how
//97	    this macro magic can possibly even work ;)
//98	@example
//99	    COUT(3) << "Some info" << std::endl;
//100	@note
//101	    The ? : operator requires both possible results to have the type of
//102	    the first. This is achieved by the int conversion operator dummy
//103	    in the OutputHandler.
#define AOUT(level) \
/*if*/ (level > ADebug::dbgLevel) ? \
        ADebug(100, "") << "" \
/*else*/ : \
        ADebug(level, Q_FUNC_INFO) << " "


#include <sys/time.h>
#include <ctime>
namespace libAeon
{
/// here's a little helper for debugging timing...

static timeval start_time = {0,0};

/// tic and toc functions work together to measure elapsed time.
/// tic saves the current time that TOC uses later to measure
/// the elapsed time. The sequence of commands:
inline void
tic()
{
  gettimeofday(&start_time, NULL);
}

inline double
toc(bool restart = false)
{
  timeval now;
  gettimeofday(&now, NULL);

  double elapsed_time = now.tv_sec - start_time.tv_sec +
                    (now.tv_usec - start_time.tv_usec)/1e6;

  if (restart)
  tic();

  return elapsed_time;
}

} // namespace libAeon

#endif


