#ifndef _MANIF_MANIF_SIM3TANGENT_H_
#define _MANIF_MANIF_SIM3TANGENT_H_

#include "manif/impl/SIM3/SIM3Tangent_base.h"

#include <Eigen/Core>

namespace manif
{
namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<SIM3Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SIM3<_Scalar>;
  using Tangent  = SIM3Tangent<_Scalar>;

  using Base = SIM3TangentBase<_Scalar>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;
  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 3, 3>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Scalar>
struct SIM3Tangent : SIM3TangentBase<SIM3Tangent<_Scalar>>
{
private:

  using Base = SIM3TangentBase<SIM3Tangent<_Scalar>>;
  using Type = SIM3Tangent<_Scalar>;

public:

  MANIF_TANGENT_TYPEDEF

  SIM3Tangent() = default;

  SIM3Tangent(const DataType& v);

  /// Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  /// SIM3Tangent specific API

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SIM3Tangent);

template <typename _Scalar>
SIM3Tangent<_Scalar>::SIM3Tangent(const DataType& theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
typename SIM3Tangent<_Scalar>::DataType&
SIM3Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename SIM3Tangent<_Scalar>::DataType&
SIM3Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SIM3TANGENT_H_ */
