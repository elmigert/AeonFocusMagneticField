// libAeon
// Copyright (C) 2010 Aeon Scientific
//
// Brad Kratochvil, Michael Kummer, Sandro Erni
//
// For licensing information see the EULA.txt file included with this
// distribution.

// $Id: autility.h 2764 2012-11-28 12:45:36Z oezcanm $

#ifndef AUTILITY_H
#define AUTILITY_H

#include <limits>

#include <armadillo>

#include <QDebug>
#include <QTextStream>

// #include <QKinematics/QSE3.h>
// #include <QKinematics/QSO3.h>



/// @defgroup utility Utility
/// This module is a collection of utilities for dealing with Qt, libAeon,
/// libQKinematics, and Armadillo.
/// @{

/// compare two matricies to see if they are equal within a specified tolerance
bool qFuzzyCompare(const arma::mat &a,
                   const arma::mat &b,
                   const qreal &tol = std::numeric_limits<qreal>::epsilon());

/// compare two vectors to see if they are equal within a specified tolerance
bool qFuzzyCompare(const arma::vec &a,
                   const arma::vec &b,
                   const qreal &tol = std::numeric_limits<qreal>::epsilon());

/// a convenience function for creating 3-element long armadillo vectors
arma::vec::fixed<3> aVector3(double a, double b, double c);
/// a convenience function for creating 6-element long armadillo vectors
arma::vec::fixed<6> aVector6(double a, double b, double c,
                             double d, double e, double f);
/*
/// converts a QVector2 into an armadillo vector
arma::vec::fixed<2> aVector2(const QVector2 &arg);
/// converts a QVector3 into an armadillo vector
arma::vec::fixed<3> aVector3(const QVector3 &arg);
/// converts a QVector6 into an armadillo vector
arma::vec::fixed<6> aVector6(const QVector6 &arg);

/// converts a QGenericVector into an armadillo vector
template<int N, typename T>
arma::vec::fixed<N> aVector(const QGenericVector<N, T> & arg)
{
  arma::vec::fixed<N> v;
  for (int i=0; i<arg.length(); ++i)
    v[i] = arg[i];
  return v;
}

/// converts a QGenericMatrix into an armadillo vector
template<int N, int M, typename T>
arma::mat::fixed<M, N> aMatrix(const QGenericMatrix<N, M, T> & arg)
{
  arma::mat::fixed<M, N> m;
  for (int r=0; r<M; ++r)
    for (int c=0; c<N; ++c)
      m(r, c) = arg(r, c);
  return m;
}

/// create a QSO3 object using only the x-axis
QSO3 axisToQSO3(const arma::vec::fixed<3> &arg);

/// converts an armadillo vector into a QVector3
QVector3 qVector3(const arma::vec::fixed<3> &arg);

/// converts an armadillo vector into a QList
QList<double> qList(const arma::vec &arg);

/// converts a 4x4 armadillo matrix into a QSE3 object
QSE3 qQSE3(const arma::mat::fixed<4,4> &m);
/// converts a QSE3 object into an armadillo matrix
arma::mat::fixed<4,4> aQSE3(const QSE3 &m);

/// converts a 3x3 armadillo matrix into a QSO3 object
QSO3 qQSO3(const arma::mat::fixed<3,3> &m);
/// converts a QSO3 object into an armadillo matrix
arma::mat::fixed<3,3> aQSO3(const QSO3 &m);

/// creates a QSO3 object out of three vector representing the axes
QSO3 qQSO3(const arma::vec::fixed<3> &x,
           const arma::vec::fixed<3> &y,
           const arma::vec::fixed<3> &z);
/// multiply a QSE3 object by a 3-vector.
/// @note this function assumes that v is a homogeneous vector and adds the
/// extra 1
arma::vec::fixed<3> operator*(const QSE3& m, const arma::vec::fixed<3>& v);
/// multiply a QSO3 object by a 3-vector.
/// @note this function assumes that v is a homogeneous vector and adds the
/// extra 1
arma::vec::fixed<3> operator*(const QSO3& m, const arma::vec::fixed<3>& v);

*/
//QSO3 aVectorToQSO3(const arma::vec::fixed<3> &arg);

/// stream operator
QDebug operator<<(QDebug dbg, const arma::mat &m);
/// stream operator
QTextStream &operator<<(QTextStream &stream, const arma::mat &m);

/// stream operator
QDebug operator<<(QDebug dbg, const arma::vec &v);
/// stream operator
QTextStream &operator<<(QTextStream &stream, const arma::vec &v);

/// replaces upper case letters with _ and lowercase for saving into xml
/// ie fooBar => foo_bar
QString manglePropertyName(const QString &val);
/// does the opposite of manglePropertyName();
QString unmanglePropertyName(const QString &val);

/// @}
// end group

#endif



