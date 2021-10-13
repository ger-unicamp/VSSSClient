// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vssref_common.proto

#ifndef PROTOBUF_vssref_5fcommon_2eproto__INCLUDED
#define PROTOBUF_vssref_5fcommon_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace VSSRef {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_vssref_5fcommon_2eproto();
void protobuf_AssignDesc_vssref_5fcommon_2eproto();
void protobuf_ShutdownFile_vssref_5fcommon_2eproto();

class Frame;
class Robot;

enum Foul {
  FREE_KICK = 0,
  PENALTY_KICK = 1,
  GOAL_KICK = 2,
  FREE_BALL = 3,
  KICKOFF = 4,
  STOP = 5,
  GAME_ON = 6,
  HALT = 7,
  Foul_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  Foul_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool Foul_IsValid(int value);
const Foul Foul_MIN = FREE_KICK;
const Foul Foul_MAX = HALT;
const int Foul_ARRAYSIZE = Foul_MAX + 1;

const ::google::protobuf::EnumDescriptor* Foul_descriptor();
inline const ::std::string& Foul_Name(Foul value) {
  return ::google::protobuf::internal::NameOfEnum(
    Foul_descriptor(), value);
}
inline bool Foul_Parse(
    const ::std::string& name, Foul* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Foul>(
    Foul_descriptor(), name, value);
}
enum Color {
  BLUE = 0,
  YELLOW = 1,
  NONE = 2,
  Color_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  Color_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool Color_IsValid(int value);
const Color Color_MIN = BLUE;
const Color Color_MAX = NONE;
const int Color_ARRAYSIZE = Color_MAX + 1;

const ::google::protobuf::EnumDescriptor* Color_descriptor();
inline const ::std::string& Color_Name(Color value) {
  return ::google::protobuf::internal::NameOfEnum(
    Color_descriptor(), value);
}
inline bool Color_Parse(
    const ::std::string& name, Color* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Color>(
    Color_descriptor(), name, value);
}
enum Quadrant {
  NO_QUADRANT = 0,
  QUADRANT_1 = 1,
  QUADRANT_2 = 2,
  QUADRANT_3 = 3,
  QUADRANT_4 = 4,
  Quadrant_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  Quadrant_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool Quadrant_IsValid(int value);
const Quadrant Quadrant_MIN = NO_QUADRANT;
const Quadrant Quadrant_MAX = QUADRANT_4;
const int Quadrant_ARRAYSIZE = Quadrant_MAX + 1;

const ::google::protobuf::EnumDescriptor* Quadrant_descriptor();
inline const ::std::string& Quadrant_Name(Quadrant value) {
  return ::google::protobuf::internal::NameOfEnum(
    Quadrant_descriptor(), value);
}
inline bool Quadrant_Parse(
    const ::std::string& name, Quadrant* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Quadrant>(
    Quadrant_descriptor(), name, value);
}
enum Half {
  NO_HALF = 0,
  FIRST_HALF = 1,
  SECOND_HALF = 2,
  Half_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  Half_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool Half_IsValid(int value);
const Half Half_MIN = NO_HALF;
const Half Half_MAX = SECOND_HALF;
const int Half_ARRAYSIZE = Half_MAX + 1;

const ::google::protobuf::EnumDescriptor* Half_descriptor();
inline const ::std::string& Half_Name(Half value) {
  return ::google::protobuf::internal::NameOfEnum(
    Half_descriptor(), value);
}
inline bool Half_Parse(
    const ::std::string& name, Half* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Half>(
    Half_descriptor(), name, value);
}
// ===================================================================

class Robot : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:VSSRef.Robot) */ {
 public:
  Robot();
  virtual ~Robot();

  Robot(const Robot& from);

  inline Robot& operator=(const Robot& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Robot& default_instance();

  void Swap(Robot* other);

  // implements Message ----------------------------------------------

  inline Robot* New() const { return New(NULL); }

  Robot* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Robot& from);
  void MergeFrom(const Robot& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Robot* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional uint32 robot_id = 1;
  void clear_robot_id();
  static const int kRobotIdFieldNumber = 1;
  ::google::protobuf::uint32 robot_id() const;
  void set_robot_id(::google::protobuf::uint32 value);

  // optional double x = 2;
  void clear_x();
  static const int kXFieldNumber = 2;
  double x() const;
  void set_x(double value);

  // optional double y = 3;
  void clear_y();
  static const int kYFieldNumber = 3;
  double y() const;
  void set_y(double value);

  // optional double orientation = 4;
  void clear_orientation();
  static const int kOrientationFieldNumber = 4;
  double orientation() const;
  void set_orientation(double value);

  // @@protoc_insertion_point(class_scope:VSSRef.Robot)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  double x_;
  double y_;
  double orientation_;
  ::google::protobuf::uint32 robot_id_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_vssref_5fcommon_2eproto();
  friend void protobuf_AssignDesc_vssref_5fcommon_2eproto();
  friend void protobuf_ShutdownFile_vssref_5fcommon_2eproto();

  void InitAsDefaultInstance();
  static Robot* default_instance_;
};
// -------------------------------------------------------------------

class Frame : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:VSSRef.Frame) */ {
 public:
  Frame();
  virtual ~Frame();

  Frame(const Frame& from);

  inline Frame& operator=(const Frame& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Frame& default_instance();

  void Swap(Frame* other);

  // implements Message ----------------------------------------------

  inline Frame* New() const { return New(NULL); }

  Frame* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Frame& from);
  void MergeFrom(const Frame& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Frame* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .VSSRef.Color teamColor = 1;
  void clear_teamcolor();
  static const int kTeamColorFieldNumber = 1;
  ::VSSRef::Color teamcolor() const;
  void set_teamcolor(::VSSRef::Color value);

  // repeated .VSSRef.Robot robots = 2;
  int robots_size() const;
  void clear_robots();
  static const int kRobotsFieldNumber = 2;
  const ::VSSRef::Robot& robots(int index) const;
  ::VSSRef::Robot* mutable_robots(int index);
  ::VSSRef::Robot* add_robots();
  ::google::protobuf::RepeatedPtrField< ::VSSRef::Robot >*
      mutable_robots();
  const ::google::protobuf::RepeatedPtrField< ::VSSRef::Robot >&
      robots() const;

  // @@protoc_insertion_point(class_scope:VSSRef.Frame)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::google::protobuf::RepeatedPtrField< ::VSSRef::Robot > robots_;
  int teamcolor_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_vssref_5fcommon_2eproto();
  friend void protobuf_AssignDesc_vssref_5fcommon_2eproto();
  friend void protobuf_ShutdownFile_vssref_5fcommon_2eproto();

  void InitAsDefaultInstance();
  static Frame* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Robot

// optional uint32 robot_id = 1;
inline void Robot::clear_robot_id() {
  robot_id_ = 0u;
}
inline ::google::protobuf::uint32 Robot::robot_id() const {
  // @@protoc_insertion_point(field_get:VSSRef.Robot.robot_id)
  return robot_id_;
}
inline void Robot::set_robot_id(::google::protobuf::uint32 value) {
  
  robot_id_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.Robot.robot_id)
}

// optional double x = 2;
inline void Robot::clear_x() {
  x_ = 0;
}
inline double Robot::x() const {
  // @@protoc_insertion_point(field_get:VSSRef.Robot.x)
  return x_;
}
inline void Robot::set_x(double value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.Robot.x)
}

// optional double y = 3;
inline void Robot::clear_y() {
  y_ = 0;
}
inline double Robot::y() const {
  // @@protoc_insertion_point(field_get:VSSRef.Robot.y)
  return y_;
}
inline void Robot::set_y(double value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.Robot.y)
}

// optional double orientation = 4;
inline void Robot::clear_orientation() {
  orientation_ = 0;
}
inline double Robot::orientation() const {
  // @@protoc_insertion_point(field_get:VSSRef.Robot.orientation)
  return orientation_;
}
inline void Robot::set_orientation(double value) {
  
  orientation_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.Robot.orientation)
}

// -------------------------------------------------------------------

// Frame

// optional .VSSRef.Color teamColor = 1;
inline void Frame::clear_teamcolor() {
  teamcolor_ = 0;
}
inline ::VSSRef::Color Frame::teamcolor() const {
  // @@protoc_insertion_point(field_get:VSSRef.Frame.teamColor)
  return static_cast< ::VSSRef::Color >(teamcolor_);
}
inline void Frame::set_teamcolor(::VSSRef::Color value) {
  
  teamcolor_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.Frame.teamColor)
}

// repeated .VSSRef.Robot robots = 2;
inline int Frame::robots_size() const {
  return robots_.size();
}
inline void Frame::clear_robots() {
  robots_.Clear();
}
inline const ::VSSRef::Robot& Frame::robots(int index) const {
  // @@protoc_insertion_point(field_get:VSSRef.Frame.robots)
  return robots_.Get(index);
}
inline ::VSSRef::Robot* Frame::mutable_robots(int index) {
  // @@protoc_insertion_point(field_mutable:VSSRef.Frame.robots)
  return robots_.Mutable(index);
}
inline ::VSSRef::Robot* Frame::add_robots() {
  // @@protoc_insertion_point(field_add:VSSRef.Frame.robots)
  return robots_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::VSSRef::Robot >*
Frame::mutable_robots() {
  // @@protoc_insertion_point(field_mutable_list:VSSRef.Frame.robots)
  return &robots_;
}
inline const ::google::protobuf::RepeatedPtrField< ::VSSRef::Robot >&
Frame::robots() const {
  // @@protoc_insertion_point(field_list:VSSRef.Frame.robots)
  return robots_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace VSSRef

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::VSSRef::Foul> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::VSSRef::Foul>() {
  return ::VSSRef::Foul_descriptor();
}
template <> struct is_proto_enum< ::VSSRef::Color> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::VSSRef::Color>() {
  return ::VSSRef::Color_descriptor();
}
template <> struct is_proto_enum< ::VSSRef::Quadrant> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::VSSRef::Quadrant>() {
  return ::VSSRef::Quadrant_descriptor();
}
template <> struct is_proto_enum< ::VSSRef::Half> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::VSSRef::Half>() {
  return ::VSSRef::Half_descriptor();
}

}  // namespace protobuf
}  // namespace google
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_vssref_5fcommon_2eproto__INCLUDED
