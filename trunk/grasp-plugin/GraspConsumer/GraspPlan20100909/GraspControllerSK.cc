// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "GraspController.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



GraspPlanStart_ptr GraspPlanStart_Helper::_nil() {
  return ::GraspPlanStart::_nil();
}

::CORBA::Boolean GraspPlanStart_Helper::is_nil(::GraspPlanStart_ptr p) {
  return ::CORBA::is_nil(p);

}

void GraspPlanStart_Helper::release(::GraspPlanStart_ptr p) {
  ::CORBA::release(p);
}

void GraspPlanStart_Helper::marshalObjRef(::GraspPlanStart_ptr obj, cdrStream& s) {
  ::GraspPlanStart::_marshalObjRef(obj, s);
}

GraspPlanStart_ptr GraspPlanStart_Helper::unmarshalObjRef(cdrStream& s) {
  return ::GraspPlanStart::_unmarshalObjRef(s);
}

void GraspPlanStart_Helper::duplicate(::GraspPlanStart_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

GraspPlanStart_ptr
GraspPlanStart::_duplicate(::GraspPlanStart_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

GraspPlanStart_ptr
GraspPlanStart::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


GraspPlanStart_ptr
GraspPlanStart::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

GraspPlanStart_ptr
GraspPlanStart::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_GraspPlanStart _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_GraspPlanStart* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_GraspPlanStart;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* GraspPlanStart::_PD_repoId = "IDL:GraspPlanStart:1.0";


_objref_GraspPlanStart::~_objref_GraspPlanStart() {
  
}


_objref_GraspPlanStart::_objref_GraspPlanStart(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::GraspPlanStart::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
_objref_GraspPlanStart::_ptrToObjRef(const char* id)
{
  if( id == ::GraspPlanStart::_PD_repoId )
    return (::GraspPlanStart_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::GraspPlanStart::_PD_repoId) )
    return (::GraspPlanStart_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  void_i_cunsigned_plong_i_cGraspPlanStart_mDblSequence3_i_cGraspPlanStart_mDblSequence9_o_cunsigned_plong
class _0RL_cd_9e8065595ff2dcff_00000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_9e8065595ff2dcff_00000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, 0, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  ::CORBA::ULong arg_0;
  GraspPlanStart::DblSequence3_var arg_1_;
  const GraspPlanStart::DblSequence3* arg_1;
  GraspPlanStart::DblSequence9_var arg_2_;
  const GraspPlanStart::DblSequence9* arg_2;
  ::CORBA::ULong arg_3;
};

void _0RL_cd_9e8065595ff2dcff_00000000::marshalArguments(cdrStream& _n)
{
  arg_0 >>= _n;
  (const GraspPlanStart::DblSequence3&) *arg_1 >>= _n;
  (const GraspPlanStart::DblSequence9&) *arg_2 >>= _n;

}

void _0RL_cd_9e8065595ff2dcff_00000000::unmarshalArguments(cdrStream& _n)
{
  (::CORBA::ULong&)arg_0 <<= _n;
  arg_1_ = new GraspPlanStart::DblSequence3;
  (GraspPlanStart::DblSequence3&)arg_1_ <<= _n;
  arg_1 = &arg_1_.in();
  arg_2_ = new GraspPlanStart::DblSequence9;
  (GraspPlanStart::DblSequence9&)arg_2_ <<= _n;
  arg_2 = &arg_2_.in();

}

void _0RL_cd_9e8065595ff2dcff_00000000::marshalReturnedValues(cdrStream& _n)
{
  arg_3 >>= _n;

}

void _0RL_cd_9e8065595ff2dcff_00000000::unmarshalReturnedValues(cdrStream& _n)
{
  (::CORBA::ULong&)arg_3 <<= _n;

}

// Local call call-back function.
static void
_0RL_lcfn_9e8065595ff2dcff_10000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_9e8065595ff2dcff_00000000* tcd = (_0RL_cd_9e8065595ff2dcff_00000000*)cd;
  _impl_GraspPlanStart* impl = (_impl_GraspPlanStart*) svnt->_ptrToInterface(GraspPlanStart::_PD_repoId);
  impl->GraspPlanningStart(tcd->arg_0, *tcd->arg_1, *tcd->arg_2, tcd->arg_3);


}

void _objref_GraspPlanStart::GraspPlanningStart(::CORBA::ULong ObjId, const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri, ::CORBA::ULong& state)
{
  _0RL_cd_9e8065595ff2dcff_00000000 _call_desc(_0RL_lcfn_9e8065595ff2dcff_10000000, "GraspPlanningStart", 19);
  _call_desc.arg_0 = ObjId;
  _call_desc.arg_1 = &(GraspPlanStart::DblSequence3&) objPos;
  _call_desc.arg_2 = &(GraspPlanStart::DblSequence9&) objOri;

  _invoke(_call_desc);
  state = _call_desc.arg_3;


}
// Local call call-back function.
static void
_0RL_lcfn_9e8065595ff2dcff_20000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_9e8065595ff2dcff_00000000* tcd = (_0RL_cd_9e8065595ff2dcff_00000000*)cd;
  _impl_GraspPlanStart* impl = (_impl_GraspPlanStart*) svnt->_ptrToInterface(GraspPlanStart::_PD_repoId);
  impl->ReleasePlanningStart(tcd->arg_0, *tcd->arg_1, *tcd->arg_2, tcd->arg_3);


}

void _objref_GraspPlanStart::ReleasePlanningStart(::CORBA::ULong ObjId, const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri, ::CORBA::ULong& state)
{
  _0RL_cd_9e8065595ff2dcff_00000000 _call_desc(_0RL_lcfn_9e8065595ff2dcff_20000000, "ReleasePlanningStart", 21);
  _call_desc.arg_0 = ObjId;
  _call_desc.arg_1 = &(GraspPlanStart::DblSequence3&) objPos;
  _call_desc.arg_2 = &(GraspPlanStart::DblSequence9&) objOri;

  _invoke(_call_desc);
  state = _call_desc.arg_3;


}
_pof_GraspPlanStart::~_pof_GraspPlanStart() {}


omniObjRef*
_pof_GraspPlanStart::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::_objref_GraspPlanStart(ior, id);
}


::CORBA::Boolean
_pof_GraspPlanStart::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::GraspPlanStart::_PD_repoId) )
    return 1;
  
  return 0;
}

const _pof_GraspPlanStart _the_pof_GraspPlanStart;

_impl_GraspPlanStart::~_impl_GraspPlanStart() {}


::CORBA::Boolean
_impl_GraspPlanStart::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "GraspPlanningStart") ) {

    _0RL_cd_9e8065595ff2dcff_00000000 _call_desc(_0RL_lcfn_9e8065595ff2dcff_10000000, "GraspPlanningStart", 19, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "ReleasePlanningStart") ) {

    _0RL_cd_9e8065595ff2dcff_00000000 _call_desc(_0RL_lcfn_9e8065595ff2dcff_20000000, "ReleasePlanningStart", 21, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
_impl_GraspPlanStart::_ptrToInterface(const char* id)
{
  if( id == ::GraspPlanStart::_PD_repoId )
    return (::_impl_GraspPlanStart*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::GraspPlanStart::_PD_repoId) )
    return (::_impl_GraspPlanStart*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
_impl_GraspPlanStart::_mostDerivedRepoId()
{
  return ::GraspPlanStart::_PD_repoId;
}

GraspPlanResult_ptr GraspPlanResult_Helper::_nil() {
  return ::GraspPlanResult::_nil();
}

::CORBA::Boolean GraspPlanResult_Helper::is_nil(::GraspPlanResult_ptr p) {
  return ::CORBA::is_nil(p);

}

void GraspPlanResult_Helper::release(::GraspPlanResult_ptr p) {
  ::CORBA::release(p);
}

void GraspPlanResult_Helper::marshalObjRef(::GraspPlanResult_ptr obj, cdrStream& s) {
  ::GraspPlanResult::_marshalObjRef(obj, s);
}

GraspPlanResult_ptr GraspPlanResult_Helper::unmarshalObjRef(cdrStream& s) {
  return ::GraspPlanResult::_unmarshalObjRef(s);
}

void GraspPlanResult_Helper::duplicate(::GraspPlanResult_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

GraspPlanResult_ptr
GraspPlanResult::_duplicate(::GraspPlanResult_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

GraspPlanResult_ptr
GraspPlanResult::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


GraspPlanResult_ptr
GraspPlanResult::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

GraspPlanResult_ptr
GraspPlanResult::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_GraspPlanResult _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_GraspPlanResult* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_GraspPlanResult;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* GraspPlanResult::_PD_repoId = "IDL:GraspPlanResult:1.0";


_objref_GraspPlanResult::~_objref_GraspPlanResult() {
  
}


_objref_GraspPlanResult::_objref_GraspPlanResult(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::GraspPlanResult::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
_objref_GraspPlanResult::_ptrToObjRef(const char* id)
{
  if( id == ::GraspPlanResult::_PD_repoId )
    return (::GraspPlanResult_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::GraspPlanResult::_PD_repoId) )
    return (::GraspPlanResult_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  void_i_cGraspPlanResult_mDblSequence3_i_cGraspPlanResult_mDblSequence9_i_cGraspPlanResult_mDblSequence3_i_cGraspPlanResult_mDblSequence9_i_cdouble_i_cunsigned_plong_o_cunsigned_plong
class _0RL_cd_9e8065595ff2dcff_30000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_9e8065595ff2dcff_30000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, 0, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

  void unmarshalReturnedValues(cdrStream&);
  void marshalReturnedValues(cdrStream&);
  
  
  GraspPlanResult::DblSequence3_var arg_0_;
  const GraspPlanResult::DblSequence3* arg_0;
  GraspPlanResult::DblSequence9_var arg_1_;
  const GraspPlanResult::DblSequence9* arg_1;
  GraspPlanResult::DblSequence3_var arg_2_;
  const GraspPlanResult::DblSequence3* arg_2;
  GraspPlanResult::DblSequence9_var arg_3_;
  const GraspPlanResult::DblSequence9* arg_3;
  ::CORBA::Double arg_4;
  ::CORBA::ULong arg_5;
  ::CORBA::ULong arg_6;
};

void _0RL_cd_9e8065595ff2dcff_30000000::marshalArguments(cdrStream& _n)
{
  (const GraspPlanResult::DblSequence3&) *arg_0 >>= _n;
  (const GraspPlanResult::DblSequence9&) *arg_1 >>= _n;
  (const GraspPlanResult::DblSequence3&) *arg_2 >>= _n;
  (const GraspPlanResult::DblSequence9&) *arg_3 >>= _n;
  arg_4 >>= _n;
  arg_5 >>= _n;

}

void _0RL_cd_9e8065595ff2dcff_30000000::unmarshalArguments(cdrStream& _n)
{
  arg_0_ = new GraspPlanResult::DblSequence3;
  (GraspPlanResult::DblSequence3&)arg_0_ <<= _n;
  arg_0 = &arg_0_.in();
  arg_1_ = new GraspPlanResult::DblSequence9;
  (GraspPlanResult::DblSequence9&)arg_1_ <<= _n;
  arg_1 = &arg_1_.in();
  arg_2_ = new GraspPlanResult::DblSequence3;
  (GraspPlanResult::DblSequence3&)arg_2_ <<= _n;
  arg_2 = &arg_2_.in();
  arg_3_ = new GraspPlanResult::DblSequence9;
  (GraspPlanResult::DblSequence9&)arg_3_ <<= _n;
  arg_3 = &arg_3_.in();
  (::CORBA::Double&)arg_4 <<= _n;
  (::CORBA::ULong&)arg_5 <<= _n;

}

void _0RL_cd_9e8065595ff2dcff_30000000::marshalReturnedValues(cdrStream& _n)
{
  arg_6 >>= _n;

}

void _0RL_cd_9e8065595ff2dcff_30000000::unmarshalReturnedValues(cdrStream& _n)
{
  (::CORBA::ULong&)arg_6 <<= _n;

}

// Local call call-back function.
static void
_0RL_lcfn_9e8065595ff2dcff_40000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_9e8065595ff2dcff_30000000* tcd = (_0RL_cd_9e8065595ff2dcff_30000000*)cd;
  _impl_GraspPlanResult* impl = (_impl_GraspPlanResult*) svnt->_ptrToInterface(GraspPlanResult::_PD_repoId);
  impl->GraspPlanningResult(*tcd->arg_0, *tcd->arg_1, *tcd->arg_2, *tcd->arg_3, tcd->arg_4, tcd->arg_5, tcd->arg_6);


}

void _objref_GraspPlanResult::GraspPlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, ::CORBA::Double angle, ::CORBA::ULong state, ::CORBA::ULong& isContinue)
{
  _0RL_cd_9e8065595ff2dcff_30000000 _call_desc(_0RL_lcfn_9e8065595ff2dcff_40000000, "GraspPlanningResult", 20);
  _call_desc.arg_0 = &(GraspPlanResult::DblSequence3&) GraspPos;
  _call_desc.arg_1 = &(GraspPlanResult::DblSequence9&) GraspOri;
  _call_desc.arg_2 = &(GraspPlanResult::DblSequence3&) ApproachPos;
  _call_desc.arg_3 = &(GraspPlanResult::DblSequence9&) ApproachOri;
  _call_desc.arg_4 = angle;
  _call_desc.arg_5 = state;

  _invoke(_call_desc);
  isContinue = _call_desc.arg_6;


}
// Local call call-back function.
static void
_0RL_lcfn_9e8065595ff2dcff_50000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_9e8065595ff2dcff_30000000* tcd = (_0RL_cd_9e8065595ff2dcff_30000000*)cd;
  _impl_GraspPlanResult* impl = (_impl_GraspPlanResult*) svnt->_ptrToInterface(GraspPlanResult::_PD_repoId);
  impl->ReleasePlanningResult(*tcd->arg_0, *tcd->arg_1, *tcd->arg_2, *tcd->arg_3, tcd->arg_4, tcd->arg_5, tcd->arg_6);


}

void _objref_GraspPlanResult::ReleasePlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, ::CORBA::Double angle, ::CORBA::ULong state, ::CORBA::ULong& isContinue)
{
  _0RL_cd_9e8065595ff2dcff_30000000 _call_desc(_0RL_lcfn_9e8065595ff2dcff_50000000, "ReleasePlanningResult", 22);
  _call_desc.arg_0 = &(GraspPlanResult::DblSequence3&) GraspPos;
  _call_desc.arg_1 = &(GraspPlanResult::DblSequence9&) GraspOri;
  _call_desc.arg_2 = &(GraspPlanResult::DblSequence3&) ApproachPos;
  _call_desc.arg_3 = &(GraspPlanResult::DblSequence9&) ApproachOri;
  _call_desc.arg_4 = angle;
  _call_desc.arg_5 = state;

  _invoke(_call_desc);
  isContinue = _call_desc.arg_6;


}
_pof_GraspPlanResult::~_pof_GraspPlanResult() {}


omniObjRef*
_pof_GraspPlanResult::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::_objref_GraspPlanResult(ior, id);
}


::CORBA::Boolean
_pof_GraspPlanResult::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::GraspPlanResult::_PD_repoId) )
    return 1;
  
  return 0;
}

const _pof_GraspPlanResult _the_pof_GraspPlanResult;

_impl_GraspPlanResult::~_impl_GraspPlanResult() {}


::CORBA::Boolean
_impl_GraspPlanResult::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "GraspPlanningResult") ) {

    _0RL_cd_9e8065595ff2dcff_30000000 _call_desc(_0RL_lcfn_9e8065595ff2dcff_40000000, "GraspPlanningResult", 20, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "ReleasePlanningResult") ) {

    _0RL_cd_9e8065595ff2dcff_30000000 _call_desc(_0RL_lcfn_9e8065595ff2dcff_50000000, "ReleasePlanningResult", 22, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
_impl_GraspPlanResult::_ptrToInterface(const char* id)
{
  if( id == ::GraspPlanResult::_PD_repoId )
    return (::_impl_GraspPlanResult*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::GraspPlanResult::_PD_repoId) )
    return (::_impl_GraspPlanResult*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
_impl_GraspPlanResult::_mostDerivedRepoId()
{
  return ::GraspPlanResult::_PD_repoId;
}

POA_GraspPlanStart::~POA_GraspPlanStart() {}

POA_GraspPlanResult::~POA_GraspPlanResult() {}

