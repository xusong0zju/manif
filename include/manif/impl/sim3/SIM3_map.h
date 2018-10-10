#ifndef _MANIF_MANIF_SIM3_MAP_H_
#define _MANIF_MANIF_SIM3_MAP_H_

#include "manif/impl/SIM3/SIM3.h"

namespace manif
{
namespace internal
{

template <typename _Scalar>
struct traits< Eigen::Map<SIM3<_Scalar>,0> >
    : public traits<SIM3<_Scalar>>
{
  using typename traits<SIM3<_Scalar>>::Scalar;
  using traits<SIM3<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const SIM3<_Scalar>,0> >
    : public traits<const SIM3<_Scalar>>
{
  using typename traits<const SIM3<_Scalar>>::Scalar;
  using traits<const SIM3<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SIM3<_Scalar>, 0>
    : public manif::SIM3Base<Map<manif::SIM3<_Scalar>, 0> >
{
  using Base = manif::SIM3Base<Map<manif::SIM3<_Scalar>, 0> >;
  using Type = Map<manif::SIM3<_Scalar>, 0>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SIM3<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SIM3<_Scalar>, 0>
    : public manif::SIM3Base<Map<const manif::SIM3<_Scalar>, 0> >
{
  using Base = manif::SIM3Base<Map<const manif::SIM3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SIM3_MAP_H_ */
