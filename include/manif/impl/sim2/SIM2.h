#ifndef _MANIF_MANIF_SIM2_H_
#define _MANIF_MANIF_SIM2_H_

#include "manif/impl/SIM2/SIM2_base.h"

#include <Eigen/Dense>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SIM2;
template <typename _Scalar> struct SIM2Tangent;

namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<SIM2<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SIM2<_Scalar>;
  using Tangent  = SIM2Tangent<_Scalar>;

  using Base = SIM2Base<SIM2<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int N       = ManifoldProperties<Base>::N;
  static constexpr int RepSize = 5;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, N, N>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Translation    = Eigen::Matrix<Scalar, Dim, 1>;
  using Vector         = Eigen::Matrix<Scalar, DoF, 1>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Scalar>
struct SIM2 : SIM2Base<SIM2<_Scalar>>
{
private:

  using Base = SIM2Base<SIM2<_Scalar>>;
  using Type = SIM2<_Scalar>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  SIM2()  = default;
  ~SIM2() = default;

  SIM2(const DataType& d);
  SIM2(const Scalar x, const Scalar y, const Scalar scale, const Scalar theta);
  SIM2(const Scalar x, const Scalar y, const Scalar scale,
       const Scalar real, const Scalar imag);

  /// Manifold common API

  const DataType& coeffs() const;

  MANIF_INHERIT_MANIFOLD_API

  /// SIM2 specific API

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::scale;
  using Base::x;
  using Base::y;

protected:

  friend class ManifoldBase<SIM2<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SIM2)

template <typename _Scalar>
SIM2<_Scalar>::SIM2(const DataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
SIM2<_Scalar>::SIM2(const Scalar x, const Scalar y,
                    const Scalar scale, const Scalar theta)
  : SIM2(DataType(x, y, scale, cos(theta), sin(theta)))
{
  using std::cos;
  using std::sin;
}

template <typename _Scalar>
SIM2<_Scalar>::SIM2(const Scalar x, const Scalar y,
                    const Scalar scale,
                    const Scalar real, const Scalar imag)
  : SIM2(DataType(x, y, scale, real, imag))
{
  //
}

template <typename _Scalar>
typename SIM2<_Scalar>::DataType&
SIM2<_Scalar>::coeffs_nonconst()
{
  return data_;
}

template <typename _Scalar>
const typename SIM2<_Scalar>::DataType&
SIM2<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SIM2_H_ */
