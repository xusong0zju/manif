#ifndef _MANIF_MANIF_SIM3_H_
#define _MANIF_MANIF_SIM3_H_

#include "manif/impl/SIM3/SIM3_base.h"

#include <Eigen/Core>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SIM3;
template <typename _Scalar> struct SIM3Tangent;

namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<SIM3<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SIM3<_Scalar>;
  using Tangent  = SIM3Tangent<_Scalar>;

  using Base = SIM3Base<SIM3<_Scalar>>;

  static constexpr int Dim = ManifoldProperties<Base>::Dim;
  static constexpr int DoF = ManifoldProperties<Base>::DoF;
  static constexpr int N   = ManifoldProperties<Base>::N;
  static constexpr int RepSize = 7;

  /// @todo would be nice to concat vec3 + quaternion
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
struct SIM3 : SIM3Base<SIM3<_Scalar>>
{
private:

  using Base = SIM3Base<SIM3<_Scalar>>;
  using Type = SIM3<_Scalar>;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MANIF_COMPLETE_MANIFOLD_TYPEDEF
  using Translation = typename Base::Translation;
  using Quaternion = Eigen::Quaternion<Scalar>;

  SIM3()  = default;
  ~SIM3() = default;

  SIM3(const DataType& d);

  SIM3(const Translation& t,
      const Eigen::Quaternion<Scalar>& q);

  SIM3(const Translation& t,
      const SO3<Scalar>& so3);

  /// Manifold common API

  const DataType& coeffs() const;

  MANIF_INHERIT_MANIFOLD_API

  /// SIM3 specific API

protected:

  friend class ManifoldBase<SIM3<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SIM3)

template <typename _Scalar>
SIM3<_Scalar>::SIM3(const DataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
SIM3<_Scalar>::SIM3(const Translation& t,
                  const Eigen::Quaternion<Scalar>& q)
  : SIM3((DataType() << t, q.coeffs() ).finished())
{
  //
}

template <typename _Scalar>
SIM3<_Scalar>::SIM3(const Translation& t,
                  const SO3<Scalar>& so3)
  : SIM3((DataType() << t, so3.coeffs() ).finished())
{
  //
}

template <typename _Scalar>
typename SIM3<_Scalar>::DataType&
SIM3<_Scalar>::coeffs_nonconst()
{
  return data_;
}

template <typename _Scalar>
const typename SIM3<_Scalar>::DataType&
SIM3<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SIM3_H_ */
