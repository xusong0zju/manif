#ifndef _MANIF_MANIF_SIM3TANGENT_MAP_H_
#define _MANIF_MANIF_SIM3TANGENT_MAP_H_

#include "manif/impl/SIM3/SIM3Tangent.h"

namespace manif
{
namespace internal
{

template <typename _Scalar>
struct traits< Eigen::Map<SIM3Tangent<_Scalar>,0> >
    : public traits<SIM3Tangent<_Scalar>>
{
  using typename traits<SIM3Tangent<_Scalar>>::Scalar;
  using traits<SIM3Tangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const SIM3Tangent<_Scalar>,0> >
    : public traits<const SIM3Tangent<_Scalar>>
{
  using typename traits<const SIM3Tangent<_Scalar>>::Scalar;
  using traits<const SIM3Tangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SIM3Tangent<_Scalar>, 0>
    : public manif::SIM3TangentBase<Map<manif::SIM3Tangent<_Scalar>, 0> >
{
  using Base = manif::SIM3TangentBase<Map<manif::SIM3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SIM3Tangent<_Scalar>, 0>
    : public manif::SIM3TangentBase<Map<const manif::SIM3Tangent<_Scalar>, 0> >
{
  using Base = manif::SIM3TangentBase<Map<const manif::SIM3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SIM3TANGENT_MAP_H_ */
