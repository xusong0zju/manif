#ifndef _MANIF_MANIF_SIM2_MAP_H_
#define _MANIF_MANIF_SIM2_MAP_H_

#include "manif/impl/SIM2/SIM2.h"

namespace manif
{
namespace internal
{

template <typename _Scalar>
struct traits< Eigen::Map<SIM2<_Scalar>,0> >
    : public traits<SIM2<_Scalar>>
{
  using typename traits<SIM2<_Scalar>>::Scalar;
  using traits<SIM2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const SIM2<_Scalar>,0> >
    : public traits<const SIM2<_Scalar>>
{
  using typename traits<const SIM2<_Scalar>>::Scalar;
  using traits<const SIM2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SIM2<_Scalar>, 0>
    : public manif::SIM2Base<Map<manif::SIM2<_Scalar>, 0> >
{
  using Base = manif::SIM2Base<Map<manif::SIM2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF
  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  friend class manif::ManifoldBase<Map<manif::SIM2<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SIM2<_Scalar>, 0>
    : public manif::SIM2Base<Map<const manif::SIM2<_Scalar>, 0> >
{
  using Base = manif::SIM2Base<Map<const manif::SIM2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF
  MANIF_INHERIT_MANIFOLD_API

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SIM2_MAP_H_ */
