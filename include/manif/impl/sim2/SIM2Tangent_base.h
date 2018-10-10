#ifndef _MANIF_MANIF_SIM2TANGENT_BASE_H_
#define _MANIF_MANIF_SIM2TANGENT_BASE_H_

#include "manif/impl/SIM2/SIM2_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct SIM2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SIM2TangentBase<_Derived>;

public:

  MANIF_TANGENT_PROPERTIES
  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;

  /// Tangent common API

  LieAlg hat() const;

  Manifold retract(OptJacobianRef J_m_t = {}) const;

  Jacobian rjac() const;
  Jacobian ljac() const;

  Jacobian adj() const;

  /// SIM2Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar scale() const;
  Scalar angle() const;
};

template <typename _Derived>
typename SIM2TangentBase<_Derived>::LieAlg
SIM2TangentBase<_Derived>::hat() const
{
  MANIF_NOT_IMPLEMENTED_YET;
  LieAlg hat = LieAlg::Zero();
  //hat.template topLeftCorner<2,2>() = todo;
}

template <typename _Derived>
typename SIM2TangentBase<_Derived>::Manifold
SIM2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  MANIF_NOT_IMPLEMENTED_YET;

  if (J_m_t)
  {
    // Jr
    (*J_m_t) = rjac();
  }

  return Manifold( A * x() - B * y(),
                   B * x() + A * y(),
                   cos_theta, sin_theta );
}

template <typename _Derived>
typename SIM2TangentBase<_Derived>::Jacobian
SIM2TangentBase<_Derived>::rjac() const
{
  using std::exp;

  Jacobian Jr = Jacobian::Identity();
  Jr.template topLeftCorner<2,2> = rotation().tranpose() * exp(-scale());

  return Jr;
}

template <typename _Derived>
typename SIM2TangentBase<_Derived>::Jacobian
SIM2TangentBase<_Derived>::ljac() const
{
  Jacobian Jl = Jacobian::Identity();

  Jl.template block<2,1,0,2>() <<  y(), -x();
  Jl.template block<2,1,0,3>() << -x(), -y();

  return Jl;
}

template <typename _Derived>
typename SIM2TangentBase<_Derived>::Jacobian
SIM2TangentBase<_Derived>::adj() const
{
  Jacobian adj = Jacobian::Zero();

  adj(0,1) = -angle();
  adj(1,0) =  angle();
  adj(0,2) =  y();
  adj(1,2) = -x();

  return adj;
}

/// SIM2Tangent specific API

template <typename _Derived>
typename SIM2TangentBase<_Derived>::Scalar
SIM2TangentBase<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SIM2TangentBase<_Derived>::Scalar
SIM2TangentBase<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SIM2TangentBase<_Derived>::Scalar
SIM2TangentBase<_Derived>::angle() const
{
  return coeffs().z();
}

template <typename _Derived>
typename SIM2TangentBase<_Derived>::Scalar
SIM2TangentBase<_Derived>::scale() const
{
  return coeffs()(3);
}



} /* namespace manif */

#endif /* _MANIF_MANIF_SIM2_BASE_H_ */
