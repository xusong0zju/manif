#ifndef _MANIF_MANIF_SIM2TANGENT_H_
#define _MANIF_MANIF_SIM2TANGENT_H_

#include "manif/impl/SIM2/SIM2Tangent_base.h"

#include <Eigen/Dense>

namespace manif
{
namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<SIM2Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SIM2<_Scalar>;
  using Tangent  = SIM2Tangent<_Scalar>;

  using Base = SIM2TangentBase<_Scalar>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, DoF, 1>;
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
struct SIM2Tangent : SIM2TangentBase<SIM2Tangent<_Scalar>>
{
private:

  using Base = SIM2TangentBase<SIM2Tangent<_Scalar>>;
  using Type = SIM2Tangent<_Scalar>;

public:

  MANIF_TANGENT_TYPEDEF

  SIM2Tangent() = default;

  SIM2Tangent(const DataType& v);
  SIM2Tangent(const Scalar x, const Scalar y,
              const Scalar scale, const Scalar theta);

  /// Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  /// SIM2Tangent specific API

  using Base::angle;
  using Base::scale;

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(SIM2Tangent);

template <typename _Scalar>
SIM2Tangent<_Scalar>::SIM2Tangent(const DataType& theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
SIM2Tangent<_Scalar>::SIM2Tangent(const Scalar x,
                                  const Scalar y,
                                  const Scalar scale,
                                  const Scalar theta)
  : SIM2Tangent(DataType(x, y, scale, theta))
{
  //
}

template <typename _Scalar>
typename SIM2Tangent<_Scalar>::DataType&
SIM2Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename SIM2Tangent<_Scalar>::DataType&
SIM2Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SIM2TANGENT_H_ */
