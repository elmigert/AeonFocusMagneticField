// libAeon
// Copyright (C) 2010 Aeon Scientific
//
// Brad Kratochvil, Michael Kummer, Sandro Erni
//
// For licensing information see the EULA.txt file included with this
// distribution.

// $Id: autility.cpp 2764 2012-11-28 12:45:36Z oezcanm $

#include "a_vector_field/autility.h"
#include "a_vector_field/adebug.h"

bool
qFuzzyCompare(const arma::mat &a, const arma::mat &b, const qreal &tol)
{
  return 0 == arma::accu(abs(a - b) >= tol);
}

bool
qFuzzyCompare(const arma::vec &a, const arma::vec &b, const qreal &tol)
{
  return 0 == arma::accu(abs(a - b) >= tol);
}


/*QVector3
qVector3(const arma::vec::fixed<3> &arg)
{
  return qVector3(arg[0], arg[1], arg[2]);
} */

QList<double>
qList(const arma::vec &arg)
{
  QList<double> list;

  arma::vec temp = arg;

  for (unsigned int i = 0; i < temp.n_elem; ++i)
    list.append(temp[i]);

  return list;
}

arma::vec::fixed<3>
aVector3(double a, double b, double c)
{
  arma::vec::fixed<3> v;
  v[0] = a;
  v[1] = b;
  v[2] = c;
  return v;
}

arma::vec::fixed<6>
aVector6(double a, double b, double c,
         double d, double e, double f)
{
  arma::vec::fixed<6> v;
  v[0] = a;
  v[1] = b;
  v[2] = c;
  v[3] = d;
  v[4] = e;
  v[5] = f;
  return v;
}
/*
arma::vec::fixed<2>
aVector2(const QVector2 & arg)
{
  arma::vec::fixed<2> v;
  for (int i=0; i<arg.length(); ++i)
    v[i] = arg[i];
  return v;
}

arma::vec::fixed<3>
aVector3(const QVector3 & arg)
{
  arma::vec::fixed<3> v;
  for (int i=0; i<arg.length(); ++i)
    v[i] = arg[i];
  return v;
}

arma::vec::fixed<6>
aVector6(const QVector6 & arg)
{
  arma::vec::fixed<6> v;
  for (int i=0; i<arg.length(); ++i)
    v[i] = arg[i];
  return v;
}

QSO3
qQSO3(const arma::mat::fixed<3,3> &m)
{
  QSO3 ret;
  for (int r=0; r<3; ++r)
    for (int c=0; c<3; ++c)
      ret(r, c) = m(r, c);
  return ret;
}

arma::mat::fixed<3,3>
aQSO3(const QSO3 &m)
{
  arma::mat::fixed<3,3> ret;
  for (int r=0; r<3; ++r)
    for (int c=0; c<3; ++c)
      ret(r, c) = m(r, c);
  return ret;
}

QSE3
qQSE3(const arma::mat::fixed<4,4> &m)
{
  QSE3 ret;
  for (int r=0; r<4; ++r)
    for (int c=0; c<4; ++c)
      ret(r, c) = m(r, c);
  return ret;
}

arma::mat::fixed<4,4>
aQSE3(const QSE3 &m)
{
  arma::mat::fixed<4,4> ret;
  for (int r=0; r<4; ++r)
    for (int c=0; c<4; ++c)
      ret(r, c) = m(r, c);
  return ret;
}

arma::vec::fixed<3>
operator*(const QSE3& m, const arma::vec::fixed<3>& v)
{
  arma::vec::fixed<3> ret;
  for (unsigned int r = 0; r < ret.n_elem; ++r)
  {
    ret[r] = m(r, 0)*v[0] + m(r, 1)*v[1] + m(r, 2)*v[2] + m(r, 3);
  }

  return ret;
}

arma::vec::fixed<3>
operator*(const QSO3& m, const arma::vec::fixed<3>& v)
{
  arma::vec::fixed<3> ret;
  for (unsigned int r = 0; r < ret.n_elem; ++r)
  {
    ret[r] = m(r, 0)*v[0] + m(r, 1)*v[1] + m(r, 2)*v[2];
  }

  return ret;
}
QSO3
qQSO3(const arma::vec::fixed<3> &x,
       const arma::vec::fixed<3> &y,
       const arma::vec::fixed<3> &z)
{
  QGenericMatrix< 3, 3, qreal > r;
  r(0,0) = x[0];
  r(1,0) = x[1];
  r(2,0) = x[2];
  r(0,1) = y[0];
  r(1,1) = y[1];
  r(2,1) = y[2];
  r(0,2) = z[0];
  r(1,2) = z[1];
  r(2,2) = z[2];

  QSO3 rot(r);
  rot.coerce();

  return rot;
}


QSO3
axisToQSO3(const arma::vec::fixed<3> &arg)
{
  arma::vec3 N = arg/arma::norm(arg, 2);
  arma::vec3 u;
  arma::vec3 v;

  const arma::vec3 x = aVector3(1, 0, 0);
  const arma::vec3 y = aVector3(0, 1, 0);

  double d = arma::dot(N, x);
  // if N and x are not orthogonal
  // then N and y are not parallel, and cross(N, y) != NULL
  if (!qFuzzyCompare(d, 0))
  {
    // by contruction u is perpendicular to both y and N
    // checking the orientation of the dot product helps us prevent orientation
    // flips
    if (d > 0)
      u = cross(N, y);
    else
      u = cross(y, N);
  }
  else // they are orthogonal
  {
    // then N and x are not parallel, and cross(N, x) != NULL
    if (arma::dot(N, y) > 0)  // TODO test me to make sure this is right
      u = cross(N, x);
    else
      u = cross(x, N);
  }

  u /= arma::norm(u, 2);

  // with u and v, we have an orthogonal basis (hamel basis) within the
  // plane normal to N
  v = cross(N, u);
  v /= arma::norm(v, 2);

  return qQSO3(N, u, v);
}
*/

QDebug
operator<<(QDebug dbg, const arma::mat &m)
{
  dbg.nospace() << "Matrix(" << m.n_rows << " x " << m.n_cols << ")";
  qreal eps = std::numeric_limits<qreal>::epsilon();

  for (unsigned int r=0; r<m.n_rows; ++r)
  {
    dbg << endl;
    dbg << "\t";
    for (unsigned int c=0; c<m.n_cols; ++c)
    {
      if (qAbs(m(r,c)) <= eps)
        dbg << 0 << "\t";
      else
        dbg << m(r, c) << "\t";
    }
  }
  return dbg.space();
}

QTextStream &
operator<<(QTextStream &stream, const arma::mat &m)
{
  stream << "Matrix(" << m.n_rows << " x " << m.n_cols << ")";
  qreal eps = std::numeric_limits<qreal>::epsilon();

  for (unsigned int r=0; r<m.n_rows; ++r)
  {
    stream << endl;
    stream << "\t";
    for (unsigned int c=0; c<m.n_cols; ++c)
    {
      if (qAbs(m(r,c)) <= eps)
        stream << 0 << "\t";
      else
        stream << m(r, c) << "\t";
    }
  }
  return stream;
}

QDebug
operator<<(QDebug dbg, const arma::vec &v)
{
  dbg.nospace() << "Vector(" << v.n_elem << ")";
  qreal eps = std::numeric_limits<qreal>::epsilon();

  dbg << endl;
  for (unsigned int i=0; i<v.n_elem; ++i)
  {
    dbg << "\t";
    if (qAbs(v[i]) <= eps)
      dbg << 0 << "\t";
    else
      dbg << v[i] << "\t";
  }
  return dbg.space();
}

QTextStream &
operator<<(QTextStream &stream, const arma::vec &v)
{
  stream << "Vector(" << v.n_elem << ")";
  qreal eps = std::numeric_limits<qreal>::epsilon();

  stream << endl;
  for (unsigned int i=0; i<v.n_elem; ++i)
  {
    stream << "\t";
    if (qAbs(v[i]) <= eps)
      stream << 0 << "\t";
    else
      stream << v[i] << "\t";
  }
  return stream;
}

QString manglePropertyName(const QString &val)
{
  QString ret;
  foreach (QChar c, val)
  {
    if (c.isUpper())
      ret += '_';

    ret += c.toLower();
  }
  return ret;
}

QString
unmanglePropertyName(const QString &val)
{
  QString ret;
  bool upperNext = false;
  foreach (QChar c, val)
  {
    if ('_' == c)
      upperNext = true;
    else
    {
      ret += (upperNext ? c.toUpper() : c);
      upperNext = false;
    }
  }
  return ret;
}


