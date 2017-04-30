// libAeon
// Copyright (C) 2010 Aeon Scientific
//
// Brad Kratochvil, Michael Kummer, Sandro Erni
//
// For licensing information see the EULA.txt file included with this
// distribution.
#pragma once
// $Id: avectorfield.h 2764 2012-11-28 12:45:36Z oezcanm $

#ifndef AVECTORFIELD_H
#define AVECTORFIELD_H

#include <QVector>
#include <QString>
#include <QObject>
#include <ros/ros.h>

#include <algorithm>
#include <armadillo>

/// @brief Vector field class
///
/// The AVectorFiled class is used to store a mapped out vector field for a coil.
/// The class includes functionality for interpolating the field value at a
/// desired location within the specified mapped out vector field, getting the
/// field at a desired location, and loading field files.
///
/// The plaintext file format is as follows:
/// @verbatim
/// total
/// points_x points_y points_z
/// min_x max_x
/// min_y max_y
/// min_z max_z
/// x1 y1 z1 B1_x B1_y B1_z
/// x2 y2 z2 B2_x B2_y B2_z
/// ...
/// @endverbatim
/// where:
/// - @b total is the total number of points in the file
/// - @b points_x, @b points_y, @b points_z are integer values for the number of
/// points along that particular axis.
/// - @b min_ and @b min_ are the minimum and maximum values along the
/// respective axes.
///
/// What follows is then the points and their field values along each axis.  All
/// units are in [m] and [T].

namespace a_vector_field{


class AVectorField //: public QObject
{ /*
  Q_OBJECT;

  ///  spacing between adjacent points in the vector field array
  Q_PROPERTY(double del READ delta WRITE setDelta)
  /// filename of the vector field
  Q_PROPERTY(QString name READ fileName WRITE setFileName)
//  /// maximum coordinates
//  Q_PROPERTY(arma::vec3 maxCoord READ max WRITE setMax)
//  /// minimum coordinates
//  Q_PROPERTY(arma::vec3 minCoord READ min WRITE setMin)
  /// true if the system is valid
  Q_PROPERTY(bool valid READ isValid WRITE setValid)
*/
  public:
    /// Default VectorField constructor.
    //AVectorField(QObject* parent = NULL);

    AVectorField(ros::NodeHandle& nodeHandle);


    /// @brief constructor that constructs a vector field by loading in a
    /// previously saved field file.
    ///
    /// @param filename Name and location of the field file
    /// @param parent the parent QObject
     // AVectorField(const QString& filename, QObject* parent = NULL);
    /// VectorField destructor.
    ~AVectorField(){};
    /// @brief Type of interpolation to use
    enum InterpolationMethod
    {
      /// nearest neighbor interpolation
      Nearest = 0,
      /// trilinear interpolation
      Trilinear,
      /// tricubic
      Tricubic
    };

    /// @name Setup
    /// @{

    /// @brief load a VectorField from file.
    ///
    /// The txt file should include
    /// mingridvalue, maxgridvalue, and deltagridvalue as its first elements.
    ///
    /// @param filename The file name and location where the txt file containing
    ///                 the data has been stored.
    bool load(const QString& filename, const QString& filepath);
    /// resets the VectorField
    void reset();
    /// @brief save VectorField to file.
    ///
    /// save a VectorField to a file.
    /// @param filename The file name and location where the data should be
    ///                 written to.
    bool save(const QString& filename);

    /// @}

    /// @name Field/Gradient Math
    /// @{

    /// @brief get the magnetic field gradient at a random location within the
    /// limits introduced by the array size.
    ///
    /// This function returns a EuclideanVector with the three components of the
    /// magnetic field gradient at a desired location.
    ///
    /// @param location The point within the limits of the magnetic vector field
    ///                 array where the gradient is to be determined.
    /// @param direction A EuclideanVector specifying the direction in which the
    ///                 gradient is to be taken.
    /// @param method the interpolation method to use
    /// @param delta is the step size we are taking the derivative over.  It is
    ///                 given as a proportion of the grid step size
    arma::vec3 gradient(const arma::vec3 &location,
                        const arma::vec3 &direction,
                        InterpolationMethod method = Trilinear,
                        double delta = 0.01) const;
    /// @brief Get the magnetic field value at a desired grid point
    ///
    /// This function returns a Vector with the magnetic field values at a
    /// requested grid point.
    ///
    /// @param index The desired grid point at which the field magnitude is
    ///                 to be evaluated.
    arma::vec3 indexedValue(const arma::ivec3 index) const;
    /// Get the magnetic field at a desired grid point
    arma::vec3 indexedValue(int x, int y, int z) const;
    /// scale the entire vector field by the amount specified
    void scale(double val);
    /// @brief This function takes an index to the vector field and converts it to a
    /// real world point point.
    arma::vec3 toCoord(const arma::ivec3 location) const;
    /// @brief This function takes an index to the vector field and converts it to a
    /// real world point point.
    arma::vec3 toCoord(int x, int y, int z) const;
    /// @brief This function takes a real WCF point and converts it to a VectorField
    /// grid point.
   arma::ivec3 toIndex(const arma::vec3 location) const;
    /// @brief This function takes a real WCF point and converts it to a VectorField
    /// grid point.
    arma::ivec3 toIndex(double x, double y, double z) const;
    /// @brief et the magnetic field value at a specified location within the
    /// limits introduced by the array size.
    ///
    /// This function returns a EuclideanVector with the three components of the
    /// magnetic field at a desired location.
    ///
    /// @param location The point within the limits of the magnetic vector field
    ///                 array.
    /// @param method the interpolation method to use
    arma::vec3 value(const arma::vec3 &location,
                     InterpolationMethod method = Nearest) const;
    /// Get the magnetic field value at a specified location
    arma::vec3 value(double x, double y, double z,
                     InterpolationMethod method = Nearest) const;
    /// @}

    /// @name Return/Set Properties
    /// @{

    /// @brief return the size of the vector field array.
    ///
    /// This function returns a vector with three integers representing the
    /// number of elements along every axis of the vector field array.
    arma::ivec3 size() const;
    /// @brief set delta
    ///
    /// spacing between adjacent points in the vector field array
    double setDelta(double val);
    /// return delta
    double delta() const;
    /// set vector field filename
    QString setFileName(QString val);
    /// return vector field file name
    const QString& fileName() const;
    /// @brief set maximum coodinates
    ///
    /// the corner point of the vector field array with the largest (most
    /// positive) values
    arma::vec3 setMax(arma::vec3 val);
    /// return maximum coordinates
    arma::vec3 max() const;
    /// @brief set minimum coodinates
    ///
    /// the corner point of the vector field array with the smallest (most
    /// negative) values
    arma::vec3 setMin(arma::vec3 val);
    /// return minimum coordinates
    arma::vec3 min() const;
    /// set true if system is valid
    bool setValid(bool val);
    /// returns true if we successfully loaded the vector field
    bool isValid() const;
    /// @}



  private:

    // resize an existing AVectorField
    //
    // Function to redefine the dimensions of a VectorField. This is used used
    // when an empty VectorField was created and afterwards data is loaded into
    // said VectorField.
    //
    // @param xsize Number of field points in the x-direction.
    // @param ysize Number of field points in the y-direction.
    // @param zsize Number of field points in the z-direction.
    bool resize(int xsize, int ysize, int zsize);

    //
    // Set the magnetic field gradient at a random location within the limits
    // introduced by the array size.
    //
    // @param location The point within the limits of the magnetic vector field
    //                 array where the gradient is to be determined.
    // @param direction A EuclideanVector specifying the direction in which the
    //                 gradient is to be taken.
    //

    void setIndexedValue(const arma::ivec3 &index, const arma::vec3& val);

    QVector<arma::cube> data;
    QVector<double> minCoord;
    QVector<double> maxCoord;


    //RoS specific Subscriber and Handle
      ros::NodeHandle nodeHandle_;
      ros::Subscriber sub_;
      ros::Publisher publ_;
      ros::Publisher vis_pub_;
      ros::ServiceServer service_;

    double gridScale;

    QString name;
    bool valid;
};

}
#endif


