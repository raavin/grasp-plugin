// This file is generated by omniidl (C++ backend) - omniORB_4_1. Do not edit.

#include "ManipulatorCommonInterface_MiddleLevel.hh"

OMNI_USING_NAMESPACE(omni)

static const char* _0RL_dyn_library_version = omniORB_4_1_dyn;

static ::CORBA::TypeCode::_Tracker _0RL_tcTrack(__FILE__);

static CORBA::TypeCode_ptr _0RL_tc_RTC_mHgMatrix = CORBA::TypeCode::PR_alias_tc("IDL:RTC/HgMatrix:1.0", "HgMatrix", CORBA::TypeCode::PR_array_tc(3, CORBA::TypeCode::PR_array_tc(4, CORBA::TypeCode::PR_double_tc(), &_0RL_tcTrack), &_0RL_tcTrack), &_0RL_tcTrack);


#if defined(HAS_Cplusplus_Namespace) && defined(_MSC_VER)
// MSVC++ does not give the constant external linkage otherwise.
namespace RTC { 
  const ::CORBA::TypeCode_ptr _tc_HgMatrix = _0RL_tc_RTC_mHgMatrix;
} 
#else
const ::CORBA::TypeCode_ptr RTC::_tc_HgMatrix = _0RL_tc_RTC_mHgMatrix;
#endif

static CORBA::TypeCode_ptr _0RL_tc_RTC_mULONG = CORBA::TypeCode::PR_alias_tc("IDL:RTC/ULONG:1.0", "ULONG", CORBA::TypeCode::PR_ulong_tc(), &_0RL_tcTrack);


static CORBA::PR_structMember _0RL_structmember_RTC_mCarPosWithElbow[] = {
  {"carPos", _0RL_tc_RTC_mHgMatrix},
  {"elbow", CORBA::TypeCode::PR_double_tc()},
  {"structFlag", _0RL_tc_RTC_mULONG}
};

#ifdef _0RL_tc_RTC_mCarPosWithElbow
#  undef _0RL_tc_RTC_mCarPosWithElbow
#endif
static CORBA::TypeCode_ptr _0RL_tc_RTC_mCarPosWithElbow = CORBA::TypeCode::PR_struct_tc("IDL:RTC/CarPosWithElbow:1.0", "CarPosWithElbow", _0RL_structmember_RTC_mCarPosWithElbow, 3, &_0RL_tcTrack);

#if defined(HAS_Cplusplus_Namespace) && defined(_MSC_VER)
// MSVC++ does not give the constant external linkage otherwise.
namespace RTC { 
  const ::CORBA::TypeCode_ptr _tc_CarPosWithElbow = _0RL_tc_RTC_mCarPosWithElbow;
} 
#else
const ::CORBA::TypeCode_ptr RTC::_tc_CarPosWithElbow = _0RL_tc_RTC_mCarPosWithElbow;
#endif


static CORBA::PR_structMember _0RL_structmember_RTC_mCartesianSpeed[] = {
  {"translation", CORBA::TypeCode::PR_double_tc()},
  {"rotation", CORBA::TypeCode::PR_double_tc()}
};

#ifdef _0RL_tc_RTC_mCartesianSpeed
#  undef _0RL_tc_RTC_mCartesianSpeed
#endif
static CORBA::TypeCode_ptr _0RL_tc_RTC_mCartesianSpeed = CORBA::TypeCode::PR_struct_tc("IDL:RTC/CartesianSpeed:1.0", "CartesianSpeed", _0RL_structmember_RTC_mCartesianSpeed, 2, &_0RL_tcTrack);

#if defined(HAS_Cplusplus_Namespace) && defined(_MSC_VER)
// MSVC++ does not give the constant external linkage otherwise.
namespace RTC { 
  const ::CORBA::TypeCode_ptr _tc_CartesianSpeed = _0RL_tc_RTC_mCartesianSpeed;
} 
#else
const ::CORBA::TypeCode_ptr RTC::_tc_CartesianSpeed = _0RL_tc_RTC_mCartesianSpeed;
#endif


const CORBA::TypeCode_ptr _tc_ManipulatorCommonInterface_Middle = CORBA::TypeCode::PR_interface_tc("IDL:ManipulatorCommonInterface_Middle:1.0", "ManipulatorCommonInterface_Middle", &_0RL_tcTrack);

static void _0RL_RTC_mHgMatrix_marshal_fn(cdrStream& _s, void* _v)
{
  RTC::HgMatrix_slice* _a = (RTC::HgMatrix_slice*)_v;
  
#ifndef OMNI_MIXED_ENDIAN_DOUBLE
  if (! _s.marshal_byte_swap()) {
    _s.put_octet_array((_CORBA_Octet*)(_a),96,omni::ALIGN_8);
  }
  else 
#endif
  {
    _s.declareArrayLength(omni::ALIGN_8, 96);
    for (_CORBA_ULong _0i0 = 0; _0i0 < 3; _0i0++){
      for (_CORBA_ULong _0i1 = 0; _0i1 < 4; _0i1++){
        _a[_0i0][_0i1] >>= _s;
      }
    }
  }

}
static void _0RL_RTC_mHgMatrix_unmarshal_fn(cdrStream& _s, void*& _v)
{
  RTC::HgMatrix_slice* _a = RTC::HgMatrix_alloc();
  _s.unmarshalArrayDouble((_CORBA_Double*)(_a), 12);

  _v = _a;
}
static void _0RL_RTC_mHgMatrix_destructor_fn(void* _v)
{
  RTC::HgMatrix_slice* _a = (RTC::HgMatrix_slice*)_v;
  RTC::HgMatrix_free(_a);
}

void operator<<=(::CORBA::Any& _a, const RTC::HgMatrix_forany& _s)
{
  RTC::HgMatrix_slice* _v;
  if (!_s.NP_nocopy())
    _v = RTC::HgMatrix_dup(_s);
  else
    _v = _s.NP_getSlice();

  _a.PR_insert(_0RL_tc_RTC_mHgMatrix,
               _0RL_RTC_mHgMatrix_marshal_fn,
               _0RL_RTC_mHgMatrix_destructor_fn,
               _v);
}
::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, RTC::HgMatrix_forany& _s)
{
  void* _v;
  if (_a.PR_extract(_0RL_tc_RTC_mHgMatrix,
                    _0RL_RTC_mHgMatrix_unmarshal_fn,
                    _0RL_RTC_mHgMatrix_marshal_fn,
                    _0RL_RTC_mHgMatrix_destructor_fn,
                    _v)) {
    _s = (RTC::HgMatrix_slice*)_v;
    return 1;
  }
  return 0;
}

static void _0RL_RTC_mCarPosWithElbow_marshal_fn(cdrStream& _s, void* _v)
{
  RTC::CarPosWithElbow* _p = (RTC::CarPosWithElbow*)_v;
  *_p >>= _s;
}
static void _0RL_RTC_mCarPosWithElbow_unmarshal_fn(cdrStream& _s, void*& _v)
{
  RTC::CarPosWithElbow* _p = new RTC::CarPosWithElbow;
  *_p <<= _s;
  _v = _p;
}
static void _0RL_RTC_mCarPosWithElbow_destructor_fn(void* _v)
{
  RTC::CarPosWithElbow* _p = (RTC::CarPosWithElbow*)_v;
  delete _p;
}

void operator<<=(::CORBA::Any& _a, const RTC::CarPosWithElbow& _s)
{
  RTC::CarPosWithElbow* _p = new RTC::CarPosWithElbow(_s);
  _a.PR_insert(_0RL_tc_RTC_mCarPosWithElbow,
               _0RL_RTC_mCarPosWithElbow_marshal_fn,
               _0RL_RTC_mCarPosWithElbow_destructor_fn,
               _p);
}
void operator<<=(::CORBA::Any& _a, RTC::CarPosWithElbow* _sp)
{
  _a.PR_insert(_0RL_tc_RTC_mCarPosWithElbow,
               _0RL_RTC_mCarPosWithElbow_marshal_fn,
               _0RL_RTC_mCarPosWithElbow_destructor_fn,
               _sp);
}

::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, RTC::CarPosWithElbow*& _sp)
{
  return _a >>= (const RTC::CarPosWithElbow*&) _sp;
}
::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, const RTC::CarPosWithElbow*& _sp)
{
  void* _v;
  if (_a.PR_extract(_0RL_tc_RTC_mCarPosWithElbow,
                    _0RL_RTC_mCarPosWithElbow_unmarshal_fn,
                    _0RL_RTC_mCarPosWithElbow_marshal_fn,
                    _0RL_RTC_mCarPosWithElbow_destructor_fn,
                    _v)) {
    _sp = (const RTC::CarPosWithElbow*)_v;
    return 1;
  }
  return 0;
}

static void _0RL_RTC_mCartesianSpeed_marshal_fn(cdrStream& _s, void* _v)
{
  RTC::CartesianSpeed* _p = (RTC::CartesianSpeed*)_v;
  *_p >>= _s;
}
static void _0RL_RTC_mCartesianSpeed_unmarshal_fn(cdrStream& _s, void*& _v)
{
  RTC::CartesianSpeed* _p = new RTC::CartesianSpeed;
  *_p <<= _s;
  _v = _p;
}
static void _0RL_RTC_mCartesianSpeed_destructor_fn(void* _v)
{
  RTC::CartesianSpeed* _p = (RTC::CartesianSpeed*)_v;
  delete _p;
}

void operator<<=(::CORBA::Any& _a, const RTC::CartesianSpeed& _s)
{
  RTC::CartesianSpeed* _p = new RTC::CartesianSpeed(_s);
  _a.PR_insert(_0RL_tc_RTC_mCartesianSpeed,
               _0RL_RTC_mCartesianSpeed_marshal_fn,
               _0RL_RTC_mCartesianSpeed_destructor_fn,
               _p);
}
void operator<<=(::CORBA::Any& _a, RTC::CartesianSpeed* _sp)
{
  _a.PR_insert(_0RL_tc_RTC_mCartesianSpeed,
               _0RL_RTC_mCartesianSpeed_marshal_fn,
               _0RL_RTC_mCartesianSpeed_destructor_fn,
               _sp);
}

::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, RTC::CartesianSpeed*& _sp)
{
  return _a >>= (const RTC::CartesianSpeed*&) _sp;
}
::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, const RTC::CartesianSpeed*& _sp)
{
  void* _v;
  if (_a.PR_extract(_0RL_tc_RTC_mCartesianSpeed,
                    _0RL_RTC_mCartesianSpeed_unmarshal_fn,
                    _0RL_RTC_mCartesianSpeed_marshal_fn,
                    _0RL_RTC_mCartesianSpeed_destructor_fn,
                    _v)) {
    _sp = (const RTC::CartesianSpeed*)_v;
    return 1;
  }
  return 0;
}

static void _0RL_ManipulatorCommonInterface__Middle_marshal_fn(cdrStream& _s, void* _v)
{
  omniObjRef* _o = (omniObjRef*)_v;
  omniObjRef::_marshal(_o, _s);
}
static void _0RL_ManipulatorCommonInterface__Middle_unmarshal_fn(cdrStream& _s, void*& _v)
{
  omniObjRef* _o = omniObjRef::_unMarshal(ManipulatorCommonInterface_Middle::_PD_repoId, _s);
  _v = _o;
}
static void _0RL_ManipulatorCommonInterface__Middle_destructor_fn(void* _v)
{
  omniObjRef* _o = (omniObjRef*)_v;
  if (_o)
    omni::releaseObjRef(_o);
}

void operator<<=(::CORBA::Any& _a, ManipulatorCommonInterface_Middle_ptr _o)
{
  ManipulatorCommonInterface_Middle_ptr _no = ManipulatorCommonInterface_Middle::_duplicate(_o);
  _a.PR_insert(_tc_ManipulatorCommonInterface_Middle,
               _0RL_ManipulatorCommonInterface__Middle_marshal_fn,
               _0RL_ManipulatorCommonInterface__Middle_destructor_fn,
               _no->_PR_getobj());
}
void operator<<=(::CORBA::Any& _a, ManipulatorCommonInterface_Middle_ptr* _op)
{
  _a.PR_insert(_tc_ManipulatorCommonInterface_Middle,
               _0RL_ManipulatorCommonInterface__Middle_marshal_fn,
               _0RL_ManipulatorCommonInterface__Middle_destructor_fn,
               (*_op)->_PR_getobj());
  *_op = ManipulatorCommonInterface_Middle::_nil();
}

::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, ManipulatorCommonInterface_Middle_ptr& _o)
{
  void* _v;
  if (_a.PR_extract(_tc_ManipulatorCommonInterface_Middle,
                    _0RL_ManipulatorCommonInterface__Middle_unmarshal_fn,
                    _0RL_ManipulatorCommonInterface__Middle_marshal_fn,
                    _0RL_ManipulatorCommonInterface__Middle_destructor_fn,
                    _v)) {
    omniObjRef* _r = (omniObjRef*)_v;
    if (_r)
      _o = (ManipulatorCommonInterface_Middle_ptr)_r->_ptrToObjRef(ManipulatorCommonInterface_Middle::_PD_repoId);
    else
      _o = ManipulatorCommonInterface_Middle::_nil();
    return 1;
  }
  return 0;
}

