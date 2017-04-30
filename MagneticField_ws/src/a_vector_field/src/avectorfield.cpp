// libAeon
// Copyright (C) 2010 Aeon Scientific
//
// Brad Kratochvil, Michael Kummer, Sandro Erni
//
// For licensing information see the EULA.txt file included with this
// distribution.

// $Id: avectorfield.cpp 2764 2012-11-28 12:45:36Z oezcanm $

#include "a_vector_field/avectorfield.h"

 #include "a_vector_field/adebug.h"
#include "a_vector_field/autility.h"

#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QDataStream>
#include <QDir>

#include <math.h>

///////////////////////////////////////////////////////////////////////////
// KEEP FUNCTIONS ALPHABETICAL
///////////////////////////////////////////////////////////////////////////
namespace a_vector_field {


/*AVectorField::AVectorField(QObject* parent) :
  QObject(parent),
  valid(false),
  gridScale(1)
{

}*/
AVectorField::AVectorField(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle),gridScale(1),valid(false)
{


   std::string fileName;
   std::string filePath;

 //Location and Name, where the file is located
   nodeHandle_.getParam("file_name", fileName);
   nodeHandle_.getParam("file_path", filePath);
   QString QName = QString::fromStdString(fileName);
   QString QPath = QString::fromStdString(filePath);
   //Control, if filePath/Name could be loaded
   if(fileName.empty() or filePath.empty()){

     ROS_INFO_STREAM("No filename or filepath");



   }
   else{
     ROS_INFO_STREAM("Loading file: " << fileName << " path: " << filePath );


     this->load(QName,QPath);

   }
/*
    IsDriving = false;
//   std_srvs::SetBool::Request = false;
//   std_srvs::SetBool::Response = false;



  sub_ = nodeHandle.subscribe(topic, queue, &HuskyHighlevelController::topicCallback, this);
  publ_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", queue);
  vis_pub_ = nodeHandle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  service_ = nodeHandle.advertiseService("HuskyMovement", &HuskyHighlevelController::MoveHusky, this);
*/


}


/*AVectorField::AVectorField(const QString& filename, QObject* parent) :
  QObject(parent)
{
  valid = load(filename);
} */


double
cubicInterpolate (const double p[4], double x)
{
  return p[1] +
      (-0.5*p[0] + 0.5*p[2])*x +
      (p[0] - 2.5*p[1] + 2.0*p[2] - 0.5*p[3])*x*x +
      (-0.5*p[0] + 1.5*p[1] - 1.5*p[2] + 0.5*p[3])*x*x*x;
}

//double //Not used; DONT DELETE!
//cubicInterpolate(const arma::vec4& p, double a) // Not used
//{
//  return p[1] +
//         (-0.5*p[0] + 0.5*p[2])*a +
//         (p[0] - 2.5*p[1] + 2.0*p[2] - 0.5*p[3])*a*a +
//         (-0.5*p[0] + 1.5*p[1] - 1.5*p[2] + 0.5*p[3])*a*a*a;
//}

double
bicubicInterpolate (const double p[4][4], double x, double y)
{
  double arr[4];
  arr[0] = cubicInterpolate(p[0], y);
  arr[1] = cubicInterpolate(p[1], y);
  arr[2] = cubicInterpolate(p[2], y);
  arr[3] = cubicInterpolate(p[3], y);
  return cubicInterpolate(arr, x);
}

//double //Not used; DONT DELETE!
//bicubicInterpolate (const arma::mat44& p, double a, double b) // Not used
//{
//  arma::vec4 arr;
//
//  arr[0] = cubicInterpolate(p.unsafe_col(0), a);
//  arr[1] = cubicInterpolate(p.unsafe_col(1), a);
//  arr[2] = cubicInterpolate(p.unsafe_col(2), a);
//  arr[3] = cubicInterpolate(p.unsafe_col(3), a);
//
//  return cubicInterpolate(arr, b);
//}

double
AVectorField::delta() const
{
  return 1.0/gridScale;
}

const QString&
AVectorField::fileName() const
{
  return name;
}

/*
arma::vec3
AVectorField::gradient(const arma::vec3 &loc,
                       const arma::vec3 &dir,
                       InterpolationMethod method,
                       double delta) const
{
  arma::vec3 unit = dir / arma::norm(dir, 2);
  double step = delta / gridScale;

  return (value(loc + step * unit, method) -
          value(loc - step * unit, method)) / (2.0 * step);
}
*/

arma::vec3
AVectorField::indexedValue(const arma::ivec3 index) const
{
  return indexedValue(index[0], index[1], index[2]);
}

arma::vec3
AVectorField::indexedValue(int x, int y, int z) const
{
  arma::vec3 ret;
// this is *really* slow in comparison!
//  ret << data[0].at(x, y, z) << data[1].at(x, y, z) << data[2].at(x, y, z);
  ret[0] = data[0].at(x, y, z);
  ret[1] = data[1].at(x, y, z);
  ret[2] = data[2].at(x, y, z);
  return ret;
}

bool
AVectorField::isValid() const
{
  return valid;
}

bool
AVectorField::load(const QString& filename, const QString& filepath)
{

  ///Getting ready for file opening
  ROS_INFO_STREAM("is loading...");
    QDir a;
   QString myPath = a.currentPath();
   ROS_INFO_STREAM("Current Path in: " <<  myPath.toUtf8().constData());
  QString OpenFile = filepath +"/" + filename + ".txt";
  ROS_INFO_STREAM("Opening Path: " << OpenFile.toUtf8().constData());

  QFileInfo info(filename);
  QFile file(OpenFile);


  //Open File - loading process
  if (file.open(QFile::ReadOnly))
  {
    ROS_INFO_STREAM("File is open");
    bool useDat = ("dat" == info.suffix());

    QTextStream txt(&file);
    QDataStream dat(&file);


    ROS_INFO_STREAM("file type: " << (useDat? "dat" : "txt" ) << endl);
   // AOUT(4) << "file type: " << (useDat? "dat" : "txt" ) << endl;

    int numberOfElements = 0;
    QVector<int> dim(3, 0);

    if (useDat)
      dat >> numberOfElements;
    else
      txt >> numberOfElements;

  //  AOUT(4) << "numberOfElements " << numberOfElements << endl;
    ROS_INFO_STREAM("numberOfElements " << numberOfElements << endl);

    if (useDat)
      dat >> dim[0]  >> dim[1] >> dim[2];
    else
      txt >> dim[0]  >> dim[1] >> dim[2];

//    AOUT(4) << "dim " << dim[0] << " " << dim[1] << " " << dim[2] << endl;
    ROS_INFO_STREAM("dim " << dim[0] << " " << dim[1] << " " << dim[2] << endl);

    reset();
    resize(dim[0], dim[1], dim[2]);

    if (useDat)
    {
      dat >> minCoord[0] >> maxCoord[0];
      dat >> minCoord[1] >> maxCoord[1];
      dat >> minCoord[2] >> maxCoord[2];
    }
    else
    {
      txt >> minCoord[0] >> maxCoord[0];
      txt >> minCoord[1] >> maxCoord[1];
      txt >> minCoord[2] >> maxCoord[2];
    }
/*
    AOUT(4) << "minCoord[0] " << minCoord[0] << " " << maxCoord[0] << endl;
    AOUT(4) << "minCoord[1] " << minCoord[1] << " " << maxCoord[1] << endl;
    AOUT(4) << "minCoord[2] " << minCoord[2] << " " << maxCoord[2] << endl;
*/

    ROS_INFO_STREAM("minCoord[0] " << minCoord[0] << " " << maxCoord[0] << endl);
    ROS_INFO_STREAM("minCoord[1] " << minCoord[1] << " " << maxCoord[1] << endl);
    ROS_INFO_STREAM("minCoord[2] " << minCoord[2] << " " << maxCoord[2] << endl);


    double scaleX = 1.0/((maxCoord[0] - minCoord[0])/(dim[0] - 1.0));
    double scaleY = 1.0/((maxCoord[1] - minCoord[1])/(dim[1] - 1.0));
    double scaleZ = 1.0/((maxCoord[2] - minCoord[2])/(dim[2] - 1.0));

 //   AOUT(4) << "scale " << scaleX << " " << scaleY << " " << scaleZ << endl;
    ROS_INFO_STREAM("scale " << scaleX << " " << scaleY << " " << scaleZ << endl);

    if (!qFuzzyCompare(scaleX, scaleY) || !qFuzzyCompare(scaleX, scaleZ) )
    {
//      AOUT(ADebug::error) << "stepsize must be equal for all axes!" << endl;
      ROS_INFO_STREAM("stepsize must be equal for all axes!" << endl);

      return false;
    }
    else
      gridScale = scaleX;

    if (qFuzzyCompare(gridScale, 0)) // catched by unequal stepsizes
    {
//      AOUT(ADebug::error) << "stepsize must be nonzero!" << endl;
      ROS_INFO_STREAM("stepsize must be nonzero!" << endl);


      return false;
    }

    for (int i=0; i< size()[0]; ++i)
    {
      for (int j=0; j< size()[1]; ++j)
      {
        for (int k=0; k< size()[2]; ++k)
        {
          arma::vec3 loc;
          arma::vec3 vec;

          if (useDat)
          {
            dat >> loc[0] >> loc[1] >> loc[2];
            dat >> vec[0] >> vec[1] >> vec[2];
          }
          else
          {
            txt >> loc[0] >> loc[1] >> loc[2];
            txt >> vec[0] >> vec[1] >> vec[2];
          }
          arma::ivec3 pos = toIndex(loc);
  /*        AOUT(10) << "loc " << loc[0] << " " << loc[1] << " " << loc[2] << endl;
          AOUT(10) << "pos " << pos[0] << " " << pos[1] << " " << pos[2] << endl;
          AOUT(10) << "pos/vec " << pos[0] << " " << pos[1] << " " << pos[2] << " "
                   << vec[0] << " " << vec[1] << " " << vec[2] << endl;
*/
          ROS_INFO_STREAM("loc " << loc[0] << " " << loc[1] << " " << loc[2] << endl);
          ROS_INFO_STREAM( "pos " << pos[0] << " " << pos[1] << " " << pos[2] << endl);
          ROS_INFO_STREAM("pos/vec " << pos[0] << " " << pos[1] << " " << pos[2] << " "
                         << vec[0] << " " << vec[1] << " " << vec[2] << endl);

          setIndexedValue(pos, vec);
        }
      }
    }
    valid = true;
  }
  else
  {
    ROS_INFO_STREAM("File could not be loaded");
 //   AOUT(0) << filename << " could not be read" << endl;
    return false;
  }

  file.close();
  name = filename;
  return true;
}


arma::vec3
AVectorField::max() const
{
  arma::vec3 ret;
  ret[0] = maxCoord[0];
  ret[1] = maxCoord[1];
  ret[2] = maxCoord[2];
  return ret;
}

arma::vec3
AVectorField::min() const
{
  arma::vec3 ret;
  ret[0] = minCoord[0];
  ret[1] = minCoord[1];
  ret[2] = minCoord[2];
  return ret;
}

void
AVectorField::reset()
{
  data.clear();
  data.resize(3);
  minCoord.resize(3);
  maxCoord.resize(3);
  valid = false;
  name = "";
}

bool
AVectorField::resize(int x_size, int y_size, int z_size)
{
  for (int i=0; i<data.size(); ++i)
  {
    data[i] = arma::cube(x_size, y_size, z_size);
    data[i].zeros();
  }
  return true;
}

bool
AVectorField::save(const QString& filename)
{
  int num = size()[0] * size()[1] * size()[2];

  QFileInfo info(filename);
  QFile file(filename);

  if (file.open(QFile::WriteOnly | QFile::Truncate))
  {
    // This would be *much* simpler if there were an AbstractStream base class,
    // but there isn't, so don't grumble about it.
    if ("dat" == info.suffix())
    {
      QDataStream out(&file);

      out << num;
      out << size()[0] << size()[1] << size()[2];
      out << minCoord[0] << maxCoord[0];
      out << minCoord[1] << maxCoord[1];
      out << minCoord[2] << maxCoord[2];

      for (int i=0; i< size()[0]; ++i)
        for (int j=0; j< size()[1]; ++j)
          for (int k=0; k< size()[2]; ++k)
          {
            arma::ivec3 loc;
            loc << i << j << k;
            arma::vec3 v = indexedValue(loc);

            arma::vec3 p = toCoord(loc);

            out << p[0] << p[1] << p[2] << v[0] << v[1] << v[2];
          }
    }
    else if ("txt" == info.suffix())
    {
      QTextStream out(&file);
      out << num << endl;
      out << size()[0] << " " << size()[1] << " " << size()[2] << endl;
      out << minCoord[0] << " " << maxCoord[0] << endl;
      out << minCoord[1] << " " << maxCoord[1] << endl;
      out << minCoord[2] << " " << maxCoord[2] << endl;

      for (int i=0; i< size()[0]; ++i)
        for (int j=0; j< size()[1]; ++j)
          for (int k=0; k< size()[2]; ++k)
          {
            arma::ivec3 loc;
            loc << i << j << k;
            arma::vec3 v = indexedValue(loc);
            arma::vec3 p = toCoord(loc);

            out << p[0] << " " << p[1] << " " << p[2] << " "
                << v[0] << " " << v[1] << " " << v[2] << endl;
          }
    }
    else
    {
   //   AOUT(ADebug::error) << "unhandled file type " << info.suffix() << endl;
      return false;
    }


  }
  file.close();
  return true;
}

void
AVectorField::scale(double val)
{
  if (qFuzzyCompare(val, 0))
 //   AOUT(ADebug::warn) << "scaling by 0" << endl;

  for (int i=0; i<data.size(); ++i)
    data[i] *= val;
}

double
AVectorField::setDelta(double val)
{
  gridScale = 1.0/val;
  return  val;
}

QString
AVectorField::setFileName(QString val)
{
  return name = val;
}

void
AVectorField::setIndexedValue(const arma::ivec3 &index, const arma::vec3& val)
{
  //{
  //}
  for (int i=0; i<3; ++i)
    data[i].at(index[0], index[1], index[2]) = val[i];
}

arma::vec3
AVectorField::setMax(const arma::vec3 val)
{
  maxCoord[0] = val[0];
  maxCoord[1] = val[1];
  maxCoord[2] = val[2];
  return  val;
}

arma::vec3
AVectorField::setMin(const arma::vec3 val)
{
  minCoord[0] = val[0];
  minCoord[1] = val[1];
  minCoord[2] = val[2];
  return  val;
}

bool
AVectorField::setValid(bool val)
{
  return valid = val;
}

arma::ivec3
AVectorField::size() const
{
  arma::ivec3 ret;
  ret[0] = data[0].n_rows;
  ret[1] = data[0].n_cols;
  ret[2] = data[0].n_slices;
  return ret;
}

arma::vec3
AVectorField::toCoord(const arma::ivec3 index) const
{
  return toCoord(index[0], index[1], index[2]);
}

arma::vec3
AVectorField::toCoord(int x, int y, int z) const
{
  arma::vec3 ret;
  ret[0] = x/gridScale + minCoord[0];
  ret[1] = y/gridScale + minCoord[1];
  ret[2] = z/gridScale + minCoord[2];
  return ret;
}

arma::ivec3
AVectorField::toIndex(const arma::vec3 location) const
{
  return toIndex(location[0], location[1], location[2]);
}

arma::ivec3
AVectorField::toIndex(double x, double y, double z) const
{
  arma::ivec3 ret;
  ret[0] = static_cast<int>(rint((x-minCoord[0])*gridScale));
  ret[1] = static_cast<int>(rint((y-minCoord[1])*gridScale));
  ret[2] = static_cast<int>(rint((z-minCoord[2])*gridScale));
  return ret;
}

double 
tricubicInterpolate (const double p[4][4][4], double x, double y, double z) 
{
  double arr[4];
  arr[0] = bicubicInterpolate(p[0], y, z);
  arr[1] = bicubicInterpolate(p[1], y, z);
  arr[2] = bicubicInterpolate(p[2], y, z);
  arr[3] = bicubicInterpolate(p[3], y, z);
  return cubicInterpolate(arr, x);
}

//double //Not used; DONT DELETE!
//tricubicInterpolate (const arma::cube::fixed<4,4,4>& p, // Not used
//                            double a, double b, double c)
//{
//  arma::vec4 arr;
//
//  arr[0] = bicubicInterpolate(p.slice(0), a, b);
//  arr[1] = bicubicInterpolate(p.slice(1), a, b);
//  arr[2] = bicubicInterpolate(p.slice(2), a, b);
//  arr[3] = bicubicInterpolate(p.slice(3), a, b);
//
//  return cubicInterpolate(arr, c);
//}
/*
arma::vec3
AVectorField::value(const arma::vec3 &location, InterpolationMethod method) const
{
  return value(location[0], location[1], location[2], method);
}
*/

arma::vec3
AVectorField::value(double x, double y, double z,
                    InterpolationMethod method) const
{
  if (x>maxCoord[0] || y>maxCoord[1] || z>maxCoord[2] ||
      x<minCoord[0] || y<minCoord[1] || z<minCoord[2])
  {
    arma::vec3 nan;
    nan << NAN << NAN << NAN;
   /*AOUT(ADebug::warn) << "requested vector field value out of bounds "
                       << x << ", " << y << ", " << z << endl;
   */ return nan;
  }


  if (Nearest == method)
  {
    return indexedValue(toIndex(x, y, z));
  }
  else if (Trilinear == method)
  {
    // trilinear interpolation for an explanation, check out
    //
    //    http://en.wikipedia.org/wiki/Trilinear_interpolation

    // scale up to our indecies, but do *not* round
    const double x_ix = (x-minCoord[0])*gridScale;
    const double y_ix = (y-minCoord[1])*gridScale;
    const double z_ix = (z-minCoord[2])*gridScale;

    // ceiling
    const int x_c = static_cast<int>(ceil(x_ix));
    const int y_c = static_cast<int>(ceil(y_ix));
    const int z_c = static_cast<int>(ceil(z_ix));

    // floor
    const int x_f = static_cast<int>(floor(x_ix));
    const int y_f = static_cast<int>(floor(y_ix));
    const int z_f = static_cast<int>(floor(z_ix));

    // delta
    const double x_d = x_ix - x_f;
    const double y_d = y_ix - y_f;
    const double z_d = z_ix - z_f;

//    // This is the original code DONT DELETE!
//    const arma::vec3 i_1 = indexedValue(x_f, y_f, z_f) * (1-z_d) +
//                           indexedValue(x_f, y_f, z_c) * z_d;
//    const arma::vec3 i_2 = indexedValue(x_f, y_c, z_f) * (1-z_d) +
//                           indexedValue(x_f, y_c, z_c) * z_d;
//    const arma::vec3 j_1 = indexedValue(x_c, y_f, z_f) * (1-z_d) +
//                           indexedValue(x_c, y_f, z_c) * z_d;
//    const arma::vec3 j_2 = indexedValue(x_c, y_c, z_f) * (1-z_d) +
//                           indexedValue(x_c, y_c, z_c) * z_d;
//
//    const arma::vec3 w_1 = i_1*(1-y_d) + i_2 * y_d;
//    const arma::vec3 w_2 = j_1*(1-y_d) + j_2 * y_d;

    // this *may* be slightly faster
    return ((indexedValue(x_f, y_f, z_f) * (1-z_d) +
             indexedValue(x_f, y_f, z_c) * z_d)*(1-y_d) +
            (indexedValue(x_f, y_c, z_f) * (1-z_d) +
             indexedValue(x_f, y_c, z_c) * z_d) * y_d)*(1-x_d) +
           ((indexedValue(x_c, y_f, z_f) * (1-z_d) +
             indexedValue(x_c, y_f, z_c) * z_d)*(1-y_d) +
            (indexedValue(x_c, y_c, z_f) * (1-z_d) +
             indexedValue(x_c, y_c, z_c) * z_d) * y_d)*x_d;
  }
  else if (Tricubic == method)
  {
    // tricubic interpolation for an explanation, check out
    //
    //    http://en.wikipedia.org/wiki/Tricubic_interpolation
    //    http://www.paulinternet.nl/?page=bicubic

    // scale up to our indecies, but do *not* round
    double x_ix = (x-minCoord[0])*gridScale;
    double y_ix = (y-minCoord[1])*gridScale;
    double z_ix = (z-minCoord[2])*gridScale;

    // floor
    const int x_f = static_cast<int>(floor(x_ix));
    const int y_f = static_cast<int>(floor(y_ix));
    const int z_f = static_cast<int>(floor(z_ix));

    // shift our target points to the smaller matrix
    x_ix -= x_f;
    y_ix -= y_f;
    z_ix -= z_f;

    arma::vec3 ret;

    for (int i=0; i<3; ++i)
    {
      // I've checked, and this memory copy takes << 10% of the execution time
      // check the interpolate function first if trying to speed up -bk
      double m[4][4][4];
      for (int x=0; x<4; ++x)
        for (int y=0; y<4; ++y)
          for (int z=0; z<4; ++z)
            m[x][y][z] = data[i].at(x_f-1+x, y_f-1+y, z_f-1+z);

      ret[i] = tricubicInterpolate(m, x_ix, y_ix, z_ix);
    }

//    // this is ~4-5x slower than the above implementation, but uses armadillo
//    arma::cube::fixed<4,4,4> sub;
//    for (int i=0; i<3; ++i)
//    {
//      sub = data[i].subcube(x_f-1, y_f-1, z_f-1, x_f+2, y_f+2, z_f+2);
//      ret[i] = tricubicInterpolate(sub, x_ix, y_ix, z_ix);
//    }

    return ret;
  }
  else // Not called, unknown method not part of AVectorField
  {
  //  AOUT(ADebug::error) << "unknown interpolation method " << method << endl;

    return arma::vec3();
  }

}

}
