// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __HIROController_hh__
#define __HIROController_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_HIROController
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_HIROController
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_HIROController
#endif






#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif





#ifndef __CommonCommands__
#define __CommonCommands__

class CommonCommands;
class _objref_CommonCommands;
class _impl_CommonCommands;

typedef _objref_CommonCommands* CommonCommands_ptr;
typedef CommonCommands_ptr CommonCommandsRef;

class CommonCommands_Helper {
public:
  typedef CommonCommands_ptr _ptr_type;

  static _ptr_type _nil();
  static _CORBA_Boolean is_nil(_ptr_type);
  static void release(_ptr_type);
  static void duplicate(_ptr_type);
  static void marshalObjRef(_ptr_type, cdrStream&);
  static _ptr_type unmarshalObjRef(cdrStream&);
};

typedef _CORBA_ObjRef_Var<_objref_CommonCommands, CommonCommands_Helper> CommonCommands_var;
typedef _CORBA_ObjRef_OUT_arg<_objref_CommonCommands,CommonCommands_Helper > CommonCommands_out;

#endif

// interface CommonCommands
class CommonCommands {
public:
  // Declarations for this interface type.
  typedef CommonCommands_ptr _ptr_type;
  typedef CommonCommands_var _var_type;

  static _ptr_type _duplicate(_ptr_type);
  static _ptr_type _narrow(::CORBA::Object_ptr);
  static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
  
  static _ptr_type _nil();

  static inline void _marshalObjRef(_ptr_type, cdrStream&);

  static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
    omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
    if (o)
      return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
    else
      return _nil();
  }

  static _core_attr const char* _PD_repoId;

  // Other IDL defined within this scope.
  struct RETURN_ID {
    typedef _CORBA_ConstrType_Variable_Var<RETURN_ID> _var_type;

    
    ::CORBA::Long id;

    ::CORBA::String_member comment;

  

    void operator>>= (cdrStream &) const;
    void operator<<= (cdrStream &);
  };

  typedef RETURN_ID::_var_type RETURN_ID_var;

  typedef _CORBA_ConstrType_Variable_OUT_arg< RETURN_ID,RETURN_ID_var > RETURN_ID_out;

  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_RETURN_ID;


};

class _objref_CommonCommands :
  public virtual ::CORBA::Object,
  public virtual omniObjRef
{
public:
  CommonCommands::RETURN_ID* servoOFF();
  CommonCommands::RETURN_ID* servoON();
  CommonCommands::RETURN_ID* servoOFFArm();
  CommonCommands::RETURN_ID* servoOFFHand();
  CommonCommands::RETURN_ID* servoONArm();
  CommonCommands::RETURN_ID* servoONHand();

  inline _objref_CommonCommands()  { _PR_setobj(0); }  // nil
  _objref_CommonCommands(omniIOR*, omniIdentity*);

protected:
  virtual ~_objref_CommonCommands();

  
private:
  virtual void* _ptrToObjRef(const char*);

  _objref_CommonCommands(const _objref_CommonCommands&);
  _objref_CommonCommands& operator = (const _objref_CommonCommands&);
  // not implemented

  friend class CommonCommands;
};

class _pof_CommonCommands : public _OMNI_NS(proxyObjectFactory) {
public:
  inline _pof_CommonCommands() : _OMNI_NS(proxyObjectFactory)(CommonCommands::_PD_repoId) {}
  virtual ~_pof_CommonCommands();

  virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
  virtual _CORBA_Boolean is_a(const char*) const;
};

class _impl_CommonCommands :
  public virtual omniServant
{
public:
  virtual ~_impl_CommonCommands();

  virtual CommonCommands::RETURN_ID* servoOFF() = 0;
  virtual CommonCommands::RETURN_ID* servoON() = 0;
  virtual CommonCommands::RETURN_ID* servoOFFArm() = 0;
  virtual CommonCommands::RETURN_ID* servoOFFHand() = 0;
  virtual CommonCommands::RETURN_ID* servoONArm() = 0;
  virtual CommonCommands::RETURN_ID* servoONHand() = 0;
  
public:  // Really protected, workaround for xlC
  virtual _CORBA_Boolean _dispatch(omniCallHandle&);

private:
  virtual void* _ptrToInterface(const char*);
  virtual const char* _mostDerivedRepoId();
  
};


_CORBA_GLOBAL_VAR _dyn_attr const ::CORBA::TypeCode_ptr _tc_CommonCommands;

#ifndef __MotionCommands__
#define __MotionCommands__

class MotionCommands;
class _objref_MotionCommands;
class _impl_MotionCommands;

typedef _objref_MotionCommands* MotionCommands_ptr;
typedef MotionCommands_ptr MotionCommandsRef;

class MotionCommands_Helper {
public:
  typedef MotionCommands_ptr _ptr_type;

  static _ptr_type _nil();
  static _CORBA_Boolean is_nil(_ptr_type);
  static void release(_ptr_type);
  static void duplicate(_ptr_type);
  static void marshalObjRef(_ptr_type, cdrStream&);
  static _ptr_type unmarshalObjRef(cdrStream&);
};

typedef _CORBA_ObjRef_Var<_objref_MotionCommands, MotionCommands_Helper> MotionCommands_var;
typedef _CORBA_ObjRef_OUT_arg<_objref_MotionCommands,MotionCommands_Helper > MotionCommands_out;

#endif

// interface MotionCommands
class MotionCommands {
public:
  // Declarations for this interface type.
  typedef MotionCommands_ptr _ptr_type;
  typedef MotionCommands_var _var_type;

  static _ptr_type _duplicate(_ptr_type);
  static _ptr_type _narrow(::CORBA::Object_ptr);
  static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
  
  static _ptr_type _nil();

  static inline void _marshalObjRef(_ptr_type, cdrStream&);

  static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
    omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
    if (o)
      return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
    else
      return _nil();
  }

  static _core_attr const char* _PD_repoId;

  // Other IDL defined within this scope.
  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_HgMatrix;

  typedef ::CORBA::Double HgMatrix[3][4];
  typedef ::CORBA::Double HgMatrix_slice[4];

  static inline HgMatrix_slice* HgMatrix_alloc() {
    return new HgMatrix_slice[3];
  }

  static inline HgMatrix_slice* HgMatrix_dup(const HgMatrix_slice* _s) {
    if (!_s) return 0;
    HgMatrix_slice* _data = HgMatrix_alloc();
    if (_data) {
      for (_CORBA_ULong _0i0 = 0; _0i0 < 3; _0i0++){
        for (_CORBA_ULong _0i1 = 0; _0i1 < 4; _0i1++){
          
          _data[_0i0][_0i1] = _s[_0i0][_0i1];

        }
      }
  
    }
    return _data;
  }

  static inline void HgMatrix_copy(HgMatrix_slice* _to, const HgMatrix_slice* _from){
    for (_CORBA_ULong _0i0 = 0; _0i0 < 3; _0i0++){
      for (_CORBA_ULong _0i1 = 0; _0i1 < 4; _0i1++){
        
        _to[_0i0][_0i1] = _from[_0i0][_0i1];

      }
    }
  
  }

  static inline void HgMatrix_free(HgMatrix_slice* _s) {
      delete [] _s;
  }

  class HgMatrix_copyHelper {
  public:
    static inline HgMatrix_slice* alloc() { return HgMatrix_alloc(); }
    static inline HgMatrix_slice* dup(const HgMatrix_slice* p) { return HgMatrix_dup(p); }
    static inline void free(HgMatrix_slice* p) { HgMatrix_free(p); }
  };

  typedef _CORBA_Array_Fix_Var<HgMatrix_copyHelper,HgMatrix_slice> HgMatrix_var;
  typedef _CORBA_Array_Fix_Forany<HgMatrix_copyHelper,HgMatrix_slice> HgMatrix_forany;

  typedef HgMatrix_slice* HgMatrix_out;

  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_DoubleSeq;

  class DoubleSeq_var;

  class DoubleSeq : public _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 >  {
  public:
    typedef DoubleSeq_var _var_type;
    inline DoubleSeq() {}
    inline DoubleSeq(const DoubleSeq& _s)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_s) {}

    inline DoubleSeq(_CORBA_ULong _max)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_max) {}
    inline DoubleSeq(_CORBA_ULong _max, _CORBA_ULong _len, ::CORBA::Double* _val, _CORBA_Boolean _rel=0)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_max, _len, _val, _rel) {}

  

    inline DoubleSeq& operator = (const DoubleSeq& _s) {
      _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > ::operator=(_s);
      return *this;
    }
  };

  class DoubleSeq_out;

  class DoubleSeq_var {
  public:
    inline DoubleSeq_var() : _pd_seq(0) {}
    inline DoubleSeq_var(DoubleSeq* _s) : _pd_seq(_s) {}
    inline DoubleSeq_var(const DoubleSeq_var& _s) {
      if( _s._pd_seq )  _pd_seq = new DoubleSeq(*_s._pd_seq);
      else              _pd_seq = 0;
    }
    inline ~DoubleSeq_var() { if( _pd_seq )  delete _pd_seq; }
      
    inline DoubleSeq_var& operator = (DoubleSeq* _s) {
      if( _pd_seq )  delete _pd_seq;
      _pd_seq = _s;
      return *this;
    }
    inline DoubleSeq_var& operator = (const DoubleSeq_var& _s) {
      if( _s._pd_seq ) {
        if( !_pd_seq )  _pd_seq = new DoubleSeq;
        *_pd_seq = *_s._pd_seq;
      } else if( _pd_seq ) {
        delete _pd_seq;
        _pd_seq = 0;
      }
      return *this;
    }
    inline ::CORBA::Double& operator [] (_CORBA_ULong _s) {
      return (*_pd_seq)[_s];
    }

  

    inline DoubleSeq* operator -> () { return _pd_seq; }
    inline const DoubleSeq* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
    inline operator DoubleSeq& () const { return *_pd_seq; }
#else
    inline operator const DoubleSeq& () const { return *_pd_seq; }
    inline operator DoubleSeq& () { return *_pd_seq; }
#endif
      
    inline const DoubleSeq& in() const { return *_pd_seq; }
    inline DoubleSeq&       inout()    { return *_pd_seq; }
    inline DoubleSeq*&      out() {
      if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
      return _pd_seq;
    }
    inline DoubleSeq* _retn() { DoubleSeq* tmp = _pd_seq; _pd_seq = 0; return tmp; }
      
    friend class DoubleSeq_out;
    
  private:
    DoubleSeq* _pd_seq;
  };

  class DoubleSeq_out {
  public:
    inline DoubleSeq_out(DoubleSeq*& _s) : _data(_s) { _data = 0; }
    inline DoubleSeq_out(DoubleSeq_var& _s)
      : _data(_s._pd_seq) { _s = (DoubleSeq*) 0; }
    inline DoubleSeq_out(const DoubleSeq_out& _s) : _data(_s._data) {}
    inline DoubleSeq_out& operator = (const DoubleSeq_out& _s) {
      _data = _s._data;
      return *this;
    }
    inline DoubleSeq_out& operator = (DoubleSeq* _s) {
      _data = _s;
      return *this;
    }
    inline operator DoubleSeq*&()  { return _data; }
    inline DoubleSeq*& ptr()       { return _data; }
    inline DoubleSeq* operator->() { return _data; }

    inline ::CORBA::Double& operator [] (_CORBA_ULong _i) {
      return (*_data)[_i];
    }

  

    DoubleSeq*& _data;

  private:
    DoubleSeq_out();
    DoubleSeq_out& operator=(const DoubleSeq_var&);
  };

  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_JointPos;

  class JointPos_var;

  class JointPos : public _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 >  {
  public:
    typedef JointPos_var _var_type;
    inline JointPos() {}
    inline JointPos(const JointPos& _s)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_s) {}

    inline JointPos(_CORBA_ULong _max)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_max) {}
    inline JointPos(_CORBA_ULong _max, _CORBA_ULong _len, ::CORBA::Double* _val, _CORBA_Boolean _rel=0)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > (_max, _len, _val, _rel) {}

  

    inline JointPos& operator = (const JointPos& _s) {
      _CORBA_Unbounded_Sequence_w_FixSizeElement< ::CORBA::Double, 8, 8 > ::operator=(_s);
      return *this;
    }
  };

  class JointPos_out;

  class JointPos_var {
  public:
    inline JointPos_var() : _pd_seq(0) {}
    inline JointPos_var(JointPos* _s) : _pd_seq(_s) {}
    inline JointPos_var(const JointPos_var& _s) {
      if( _s._pd_seq )  _pd_seq = new JointPos(*_s._pd_seq);
      else              _pd_seq = 0;
    }
    inline ~JointPos_var() { if( _pd_seq )  delete _pd_seq; }
      
    inline JointPos_var& operator = (JointPos* _s) {
      if( _pd_seq )  delete _pd_seq;
      _pd_seq = _s;
      return *this;
    }
    inline JointPos_var& operator = (const JointPos_var& _s) {
      if( _s._pd_seq ) {
        if( !_pd_seq )  _pd_seq = new JointPos;
        *_pd_seq = *_s._pd_seq;
      } else if( _pd_seq ) {
        delete _pd_seq;
        _pd_seq = 0;
      }
      return *this;
    }
    inline ::CORBA::Double& operator [] (_CORBA_ULong _s) {
      return (*_pd_seq)[_s];
    }

  

    inline JointPos* operator -> () { return _pd_seq; }
    inline const JointPos* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
    inline operator JointPos& () const { return *_pd_seq; }
#else
    inline operator const JointPos& () const { return *_pd_seq; }
    inline operator JointPos& () { return *_pd_seq; }
#endif
      
    inline const JointPos& in() const { return *_pd_seq; }
    inline JointPos&       inout()    { return *_pd_seq; }
    inline JointPos*&      out() {
      if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
      return _pd_seq;
    }
    inline JointPos* _retn() { JointPos* tmp = _pd_seq; _pd_seq = 0; return tmp; }
      
    friend class JointPos_out;
    
  private:
    JointPos* _pd_seq;
  };

  class JointPos_out {
  public:
    inline JointPos_out(JointPos*& _s) : _data(_s) { _data = 0; }
    inline JointPos_out(JointPos_var& _s)
      : _data(_s._pd_seq) { _s = (JointPos*) 0; }
    inline JointPos_out(const JointPos_out& _s) : _data(_s._data) {}
    inline JointPos_out& operator = (const JointPos_out& _s) {
      _data = _s._data;
      return *this;
    }
    inline JointPos_out& operator = (JointPos* _s) {
      _data = _s;
      return *this;
    }
    inline operator JointPos*&()  { return _data; }
    inline JointPos*& ptr()       { return _data; }
    inline JointPos* operator->() { return _data; }

    inline ::CORBA::Double& operator [] (_CORBA_ULong _i) {
      return (*_data)[_i];
    }

  

    JointPos*& _data;

  private:
    JointPos_out();
    JointPos_out& operator=(const JointPos_var&);
  };

  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_JointPosSeq;

  class JointPosSeq_var;

  class JointPosSeq : public _CORBA_Unbounded_Sequence< JointPos >  {
  public:
    typedef JointPosSeq_var _var_type;
    inline JointPosSeq() {}
    inline JointPosSeq(const JointPosSeq& _s)
      : _CORBA_Unbounded_Sequence< JointPos > (_s) {}

    inline JointPosSeq(_CORBA_ULong _max)
      : _CORBA_Unbounded_Sequence< JointPos > (_max) {}
    inline JointPosSeq(_CORBA_ULong _max, _CORBA_ULong _len, JointPos* _val, _CORBA_Boolean _rel=0)
      : _CORBA_Unbounded_Sequence< JointPos > (_max, _len, _val, _rel) {}

  

    inline JointPosSeq& operator = (const JointPosSeq& _s) {
      _CORBA_Unbounded_Sequence< JointPos > ::operator=(_s);
      return *this;
    }
  };

  class JointPosSeq_out;

  class JointPosSeq_var {
  public:
    inline JointPosSeq_var() : _pd_seq(0) {}
    inline JointPosSeq_var(JointPosSeq* _s) : _pd_seq(_s) {}
    inline JointPosSeq_var(const JointPosSeq_var& _s) {
      if( _s._pd_seq )  _pd_seq = new JointPosSeq(*_s._pd_seq);
      else              _pd_seq = 0;
    }
    inline ~JointPosSeq_var() { if( _pd_seq )  delete _pd_seq; }
      
    inline JointPosSeq_var& operator = (JointPosSeq* _s) {
      if( _pd_seq )  delete _pd_seq;
      _pd_seq = _s;
      return *this;
    }
    inline JointPosSeq_var& operator = (const JointPosSeq_var& _s) {
      if( _s._pd_seq ) {
        if( !_pd_seq )  _pd_seq = new JointPosSeq;
        *_pd_seq = *_s._pd_seq;
      } else if( _pd_seq ) {
        delete _pd_seq;
        _pd_seq = 0;
      }
      return *this;
    }
    inline JointPos& operator [] (_CORBA_ULong _s) {
      return (*_pd_seq)[_s];
    }

  

    inline JointPosSeq* operator -> () { return _pd_seq; }
    inline const JointPosSeq* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
    inline operator JointPosSeq& () const { return *_pd_seq; }
#else
    inline operator const JointPosSeq& () const { return *_pd_seq; }
    inline operator JointPosSeq& () { return *_pd_seq; }
#endif
      
    inline const JointPosSeq& in() const { return *_pd_seq; }
    inline JointPosSeq&       inout()    { return *_pd_seq; }
    inline JointPosSeq*&      out() {
      if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
      return _pd_seq;
    }
    inline JointPosSeq* _retn() { JointPosSeq* tmp = _pd_seq; _pd_seq = 0; return tmp; }
      
    friend class JointPosSeq_out;
    
  private:
    JointPosSeq* _pd_seq;
  };

  class JointPosSeq_out {
  public:
    inline JointPosSeq_out(JointPosSeq*& _s) : _data(_s) { _data = 0; }
    inline JointPosSeq_out(JointPosSeq_var& _s)
      : _data(_s._pd_seq) { _s = (JointPosSeq*) 0; }
    inline JointPosSeq_out(const JointPosSeq_out& _s) : _data(_s._data) {}
    inline JointPosSeq_out& operator = (const JointPosSeq_out& _s) {
      _data = _s._data;
      return *this;
    }
    inline JointPosSeq_out& operator = (JointPosSeq* _s) {
      _data = _s;
      return *this;
    }
    inline operator JointPosSeq*&()  { return _data; }
    inline JointPosSeq*& ptr()       { return _data; }
    inline JointPosSeq* operator->() { return _data; }

    inline JointPos& operator [] (_CORBA_ULong _i) {
      return (*_data)[_i];
    }

  

    JointPosSeq*& _data;

  private:
    JointPosSeq_out();
    JointPosSeq_out& operator=(const JointPosSeq_var&);
  };

  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_ULONG;

  typedef ::CORBA::ULong ULONG;
  typedef ::CORBA::ULong_out ULONG_out;

  struct CarPosWithElbow {
    typedef _CORBA_ConstrType_Fix_Var<CarPosWithElbow> _var_type;

    
    HgMatrix carPos;

    ::CORBA::Double elbow;

    ULONG structFlag;

  

    void operator>>= (cdrStream &) const;
    void operator<<= (cdrStream &);
  };

  typedef CarPosWithElbow::_var_type CarPosWithElbow_var;

  typedef CarPosWithElbow& CarPosWithElbow_out;

  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_CarPosWithElbow;

  struct RETURN_ID {
    typedef _CORBA_ConstrType_Variable_Var<RETURN_ID> _var_type;

    
    ::CORBA::Long id;

    ::CORBA::String_member comment;

  

    void operator>>= (cdrStream &) const;
    void operator<<= (cdrStream &);
  };

  typedef RETURN_ID::_var_type RETURN_ID_var;

  typedef _CORBA_ConstrType_Variable_OUT_arg< RETURN_ID,RETURN_ID_var > RETURN_ID_out;

  static _dyn_attr const ::CORBA::TypeCode_ptr _tc_RETURN_ID;


};

class _objref_MotionCommands :
  public virtual ::CORBA::Object,
  public virtual omniObjRef
{
public:
  MotionCommands::RETURN_ID* closeGripper();
  MotionCommands::RETURN_ID* moveGripper(const MotionCommands::DoubleSeq& r_angle, const MotionCommands::DoubleSeq& l_angle);
  MotionCommands::RETURN_ID* moveLinearCartesianAbs(const MotionCommands::CarPosWithElbow& rArm, const MotionCommands::CarPosWithElbow& lArm);
  MotionCommands::RETURN_ID* moveLinearCartesianRel(const MotionCommands::CarPosWithElbow& rArm, const MotionCommands::CarPosWithElbow& lArm);
  MotionCommands::RETURN_ID* movePTPJointAbs(const MotionCommands::JointPos& jointPoints);
  MotionCommands::RETURN_ID* movePTPJointRel(const MotionCommands::JointPos& jointPoints);
  MotionCommands::RETURN_ID* movePTPJointAbsSeq(const MotionCommands::JointPosSeq& jointPointsSeq, const MotionCommands::DoubleSeq& timeSeq);
  MotionCommands::RETURN_ID* openGripper();
  MotionCommands::RETURN_ID* setSpeedCartesian(MotionCommands::ULONG spdRatio);
  MotionCommands::RETURN_ID* setSpeedJoint(MotionCommands::ULONG spdRatio);

  inline _objref_MotionCommands()  { _PR_setobj(0); }  // nil
  _objref_MotionCommands(omniIOR*, omniIdentity*);

protected:
  virtual ~_objref_MotionCommands();

  
private:
  virtual void* _ptrToObjRef(const char*);

  _objref_MotionCommands(const _objref_MotionCommands&);
  _objref_MotionCommands& operator = (const _objref_MotionCommands&);
  // not implemented

  friend class MotionCommands;
};

class _pof_MotionCommands : public _OMNI_NS(proxyObjectFactory) {
public:
  inline _pof_MotionCommands() : _OMNI_NS(proxyObjectFactory)(MotionCommands::_PD_repoId) {}
  virtual ~_pof_MotionCommands();

  virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
  virtual _CORBA_Boolean is_a(const char*) const;
};

class _impl_MotionCommands :
  public virtual omniServant
{
public:
  virtual ~_impl_MotionCommands();

  virtual MotionCommands::RETURN_ID* closeGripper() = 0;
  virtual MotionCommands::RETURN_ID* moveGripper(const MotionCommands::DoubleSeq& r_angle, const MotionCommands::DoubleSeq& l_angle) = 0;
  virtual MotionCommands::RETURN_ID* moveLinearCartesianAbs(const MotionCommands::CarPosWithElbow& rArm, const MotionCommands::CarPosWithElbow& lArm) = 0;
  virtual MotionCommands::RETURN_ID* moveLinearCartesianRel(const MotionCommands::CarPosWithElbow& rArm, const MotionCommands::CarPosWithElbow& lArm) = 0;
  virtual MotionCommands::RETURN_ID* movePTPJointAbs(const MotionCommands::JointPos& jointPoints) = 0;
  virtual MotionCommands::RETURN_ID* movePTPJointRel(const MotionCommands::JointPos& jointPoints) = 0;
  virtual MotionCommands::RETURN_ID* movePTPJointAbsSeq(const MotionCommands::JointPosSeq& jointPointsSeq, const MotionCommands::DoubleSeq& timeSeq) = 0;
  virtual MotionCommands::RETURN_ID* openGripper() = 0;
  virtual MotionCommands::RETURN_ID* setSpeedCartesian(MotionCommands::ULONG spdRatio) = 0;
  virtual MotionCommands::RETURN_ID* setSpeedJoint(MotionCommands::ULONG spdRatio) = 0;
  
public:  // Really protected, workaround for xlC
  virtual _CORBA_Boolean _dispatch(omniCallHandle&);

private:
  virtual void* _ptrToInterface(const char*);
  virtual const char* _mostDerivedRepoId();
  
};


_CORBA_GLOBAL_VAR _dyn_attr const ::CORBA::TypeCode_ptr _tc_MotionCommands;



class POA_CommonCommands :
  public virtual _impl_CommonCommands,
  public virtual ::PortableServer::ServantBase
{
public:
  virtual ~POA_CommonCommands();

  inline ::CommonCommands_ptr _this() {
    return (::CommonCommands_ptr) _do_this(::CommonCommands::_PD_repoId);
  }
};

class POA_MotionCommands :
  public virtual _impl_MotionCommands,
  public virtual ::PortableServer::ServantBase
{
public:
  virtual ~POA_MotionCommands();

  inline ::MotionCommands_ptr _this() {
    return (::MotionCommands_ptr) _do_this(::MotionCommands::_PD_repoId);
  }
};







#undef _core_attr
#undef _dyn_attr

extern void operator<<=(::CORBA::Any& _a, const CommonCommands::RETURN_ID& _s);
extern void operator<<=(::CORBA::Any& _a, CommonCommands::RETURN_ID* _sp);
extern _CORBA_Boolean operator>>=(const ::CORBA::Any& _a, CommonCommands::RETURN_ID*& _sp);
extern _CORBA_Boolean operator>>=(const ::CORBA::Any& _a, const CommonCommands::RETURN_ID*& _sp);

void operator<<=(::CORBA::Any& _a, CommonCommands_ptr _s);
void operator<<=(::CORBA::Any& _a, CommonCommands_ptr* _s);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, CommonCommands_ptr& _s);

void operator<<=(::CORBA::Any& _a, const MotionCommands::HgMatrix_forany& _s);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, MotionCommands::HgMatrix_forany& _s);

void operator<<=(::CORBA::Any& _a, const MotionCommands::DoubleSeq& _s);
void operator<<=(::CORBA::Any& _a, MotionCommands::DoubleSeq* _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, MotionCommands::DoubleSeq*& _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, const MotionCommands::DoubleSeq*& _sp);

void operator<<=(::CORBA::Any& _a, const MotionCommands::JointPos& _s);
void operator<<=(::CORBA::Any& _a, MotionCommands::JointPos* _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, MotionCommands::JointPos*& _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, const MotionCommands::JointPos*& _sp);

void operator<<=(::CORBA::Any& _a, const MotionCommands::JointPosSeq& _s);
void operator<<=(::CORBA::Any& _a, MotionCommands::JointPosSeq* _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, MotionCommands::JointPosSeq*& _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, const MotionCommands::JointPosSeq*& _sp);

extern void operator<<=(::CORBA::Any& _a, const MotionCommands::CarPosWithElbow& _s);
extern void operator<<=(::CORBA::Any& _a, MotionCommands::CarPosWithElbow* _sp);
extern _CORBA_Boolean operator>>=(const ::CORBA::Any& _a, MotionCommands::CarPosWithElbow*& _sp);
extern _CORBA_Boolean operator>>=(const ::CORBA::Any& _a, const MotionCommands::CarPosWithElbow*& _sp);

extern void operator<<=(::CORBA::Any& _a, const MotionCommands::RETURN_ID& _s);
extern void operator<<=(::CORBA::Any& _a, MotionCommands::RETURN_ID* _sp);
extern _CORBA_Boolean operator>>=(const ::CORBA::Any& _a, MotionCommands::RETURN_ID*& _sp);
extern _CORBA_Boolean operator>>=(const ::CORBA::Any& _a, const MotionCommands::RETURN_ID*& _sp);

void operator<<=(::CORBA::Any& _a, MotionCommands_ptr _s);
void operator<<=(::CORBA::Any& _a, MotionCommands_ptr* _s);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, MotionCommands_ptr& _s);



inline void
CommonCommands::_marshalObjRef(::CommonCommands_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}


inline void
MotionCommands::_marshalObjRef(::MotionCommands_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_HIROController
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_HIROController
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_HIROController
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_HIROController
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_HIROController
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_HIROController
#endif

#endif  // __HIROController_hh__

