// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vssref_command.proto

#ifndef PROTOBUF_INCLUDED_vssref_5fcommand_2eproto
#define PROTOBUF_INCLUDED_vssref_5fcommand_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "vssref_common.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_vssref_5fcommand_2eproto 

namespace protobuf_vssref_5fcommand_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_vssref_5fcommand_2eproto
namespace VSSRef {
namespace ref_to_team {
class VSSRef_Command;
class VSSRef_CommandDefaultTypeInternal;
extern VSSRef_CommandDefaultTypeInternal _VSSRef_Command_default_instance_;
}  // namespace ref_to_team
}  // namespace VSSRef
namespace google {
namespace protobuf {
template<> ::VSSRef::ref_to_team::VSSRef_Command* Arena::CreateMaybeMessage<::VSSRef::ref_to_team::VSSRef_Command>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace VSSRef {
namespace ref_to_team {

// ===================================================================

class VSSRef_Command : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:VSSRef.ref_to_team.VSSRef_Command) */ {
 public:
  VSSRef_Command();
  virtual ~VSSRef_Command();

  VSSRef_Command(const VSSRef_Command& from);

  inline VSSRef_Command& operator=(const VSSRef_Command& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  VSSRef_Command(VSSRef_Command&& from) noexcept
    : VSSRef_Command() {
    *this = ::std::move(from);
  }

  inline VSSRef_Command& operator=(VSSRef_Command&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const VSSRef_Command& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const VSSRef_Command* internal_default_instance() {
    return reinterpret_cast<const VSSRef_Command*>(
               &_VSSRef_Command_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(VSSRef_Command* other);
  friend void swap(VSSRef_Command& a, VSSRef_Command& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline VSSRef_Command* New() const final {
    return CreateMaybeMessage<VSSRef_Command>(NULL);
  }

  VSSRef_Command* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<VSSRef_Command>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const VSSRef_Command& from);
  void MergeFrom(const VSSRef_Command& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(VSSRef_Command* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // .VSSRef.Foul foul = 1;
  void clear_foul();
  static const int kFoulFieldNumber = 1;
  ::VSSRef::Foul foul() const;
  void set_foul(::VSSRef::Foul value);

  // .VSSRef.Color teamcolor = 2;
  void clear_teamcolor();
  static const int kTeamcolorFieldNumber = 2;
  ::VSSRef::Color teamcolor() const;
  void set_teamcolor(::VSSRef::Color value);

  // double timestamp = 4;
  void clear_timestamp();
  static const int kTimestampFieldNumber = 4;
  double timestamp() const;
  void set_timestamp(double value);

  // .VSSRef.Quadrant foulQuadrant = 3;
  void clear_foulquadrant();
  static const int kFoulQuadrantFieldNumber = 3;
  ::VSSRef::Quadrant foulquadrant() const;
  void set_foulquadrant(::VSSRef::Quadrant value);

  // .VSSRef.Half gameHalf = 5;
  void clear_gamehalf();
  static const int kGameHalfFieldNumber = 5;
  ::VSSRef::Half gamehalf() const;
  void set_gamehalf(::VSSRef::Half value);

  // @@protoc_insertion_point(class_scope:VSSRef.ref_to_team.VSSRef_Command)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  int foul_;
  int teamcolor_;
  double timestamp_;
  int foulquadrant_;
  int gamehalf_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_vssref_5fcommand_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// VSSRef_Command

// .VSSRef.Foul foul = 1;
inline void VSSRef_Command::clear_foul() {
  foul_ = 0;
}
inline ::VSSRef::Foul VSSRef_Command::foul() const {
  // @@protoc_insertion_point(field_get:VSSRef.ref_to_team.VSSRef_Command.foul)
  return static_cast< ::VSSRef::Foul >(foul_);
}
inline void VSSRef_Command::set_foul(::VSSRef::Foul value) {
  
  foul_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.ref_to_team.VSSRef_Command.foul)
}

// .VSSRef.Color teamcolor = 2;
inline void VSSRef_Command::clear_teamcolor() {
  teamcolor_ = 0;
}
inline ::VSSRef::Color VSSRef_Command::teamcolor() const {
  // @@protoc_insertion_point(field_get:VSSRef.ref_to_team.VSSRef_Command.teamcolor)
  return static_cast< ::VSSRef::Color >(teamcolor_);
}
inline void VSSRef_Command::set_teamcolor(::VSSRef::Color value) {
  
  teamcolor_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.ref_to_team.VSSRef_Command.teamcolor)
}

// .VSSRef.Quadrant foulQuadrant = 3;
inline void VSSRef_Command::clear_foulquadrant() {
  foulquadrant_ = 0;
}
inline ::VSSRef::Quadrant VSSRef_Command::foulquadrant() const {
  // @@protoc_insertion_point(field_get:VSSRef.ref_to_team.VSSRef_Command.foulQuadrant)
  return static_cast< ::VSSRef::Quadrant >(foulquadrant_);
}
inline void VSSRef_Command::set_foulquadrant(::VSSRef::Quadrant value) {
  
  foulquadrant_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.ref_to_team.VSSRef_Command.foulQuadrant)
}

// double timestamp = 4;
inline void VSSRef_Command::clear_timestamp() {
  timestamp_ = 0;
}
inline double VSSRef_Command::timestamp() const {
  // @@protoc_insertion_point(field_get:VSSRef.ref_to_team.VSSRef_Command.timestamp)
  return timestamp_;
}
inline void VSSRef_Command::set_timestamp(double value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.ref_to_team.VSSRef_Command.timestamp)
}

// .VSSRef.Half gameHalf = 5;
inline void VSSRef_Command::clear_gamehalf() {
  gamehalf_ = 0;
}
inline ::VSSRef::Half VSSRef_Command::gamehalf() const {
  // @@protoc_insertion_point(field_get:VSSRef.ref_to_team.VSSRef_Command.gameHalf)
  return static_cast< ::VSSRef::Half >(gamehalf_);
}
inline void VSSRef_Command::set_gamehalf(::VSSRef::Half value) {
  
  gamehalf_ = value;
  // @@protoc_insertion_point(field_set:VSSRef.ref_to_team.VSSRef_Command.gameHalf)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace ref_to_team
}  // namespace VSSRef

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_vssref_5fcommand_2eproto
