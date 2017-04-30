// libAeon
// Copyright (C) 2010 Aeon Scientific
//
// Brad Kratochvil, Michael Kummer, Sandro Erni
//
// For licensing information see the EULA.txt file included with this
// distribution.

// $Id: adebug.cpp 2764 2012-11-28 12:45:36Z oezcanm $

#include "a_vector_field/adebug.h"
#ifdef QTVERSION_5
#include <QtWidgets/QMessageBox>
#else
#include <QMessageBox>
#endif

#include <syslog.h>

int ADebug::msgCritical = -2;
int ADebug::msgWarning = -1;
int ADebug::error = 0;
int ADebug::warn = 1;
int ADebug::dbgLevel = 2;
int ADebug::all = 10;

QTextStream ADebug::derr(stderr, QIODevice::WriteOnly);
QMutex ADebug::lock;
const char* ADebug::logName="libAeon";
bool ADebug::syslogOpen = false;
bool ADebug::writeSysLog = true;
bool ADebug::writeStdErr = false;

ADebug::ADebug() :
  QTextStream(&str),
  id(""),
  level(0)
{
}

ADebug::ADebug(int l, const QString &text) :
  QTextStream(&str),
  id(text),
  level(l)
{
}

ADebug::~ADebug()
{
  if (level <= dbgLevel)
  {
    QMutexLocker locker(&lock);

    if (writeStdErr)
    {
      derr << (!id.isEmpty() ? "[" + id + "]" : "" ) + str;
      derr.flush();
    }

    if (writeSysLog)
      writeLog((!id.isEmpty() ? "[" + id + "]" : "" ) + str);
  }
}

void
ADebug::openLog()
{
  if (!syslogOpen)
  {
    openlog(logName, LOG_CONS, LOG_USER);
    syslogOpen = true;
  }
}

void
ADebug::writeLog(const QString& val)
{
  openLog();
#ifdef QTVERSION_5
  syslog(LOG_INFO, val.toLatin1());
#else
  syslog(LOG_INFO, val.toAscii());
#endif
  closeLog();
}

void
ADebug::closeLog()
{
  if (syslogOpen)
  {
    closelog();
    syslogOpen = false;
  }
}

void
ADebug::setDebugLevel(int level)
{
  dbgLevel = level;
}

int
ADebug::debugLevel()
{
  return dbgLevel;
}

