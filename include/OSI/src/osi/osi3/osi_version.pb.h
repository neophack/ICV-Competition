// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: osi_version.proto

#ifndef PROTOBUF_osi_5fversion_2eproto__INCLUDED
#define PROTOBUF_osi_5fversion_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/descriptor.pb.h>
// @@protoc_insertion_point(includes)

namespace protobuf_osi_5fversion_2eproto {
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
void InitDefaultsInterfaceVersionImpl();
void InitDefaultsInterfaceVersion();
inline void InitDefaults() {
  InitDefaultsInterfaceVersion();
}
}  // namespace protobuf_osi_5fversion_2eproto
namespace osi3 {
class InterfaceVersion;
class InterfaceVersionDefaultTypeInternal;
extern InterfaceVersionDefaultTypeInternal _InterfaceVersion_default_instance_;
}  // namespace osi3
namespace osi3 {

// ===================================================================

class InterfaceVersion : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:osi3.InterfaceVersion) */ {
 public:
  InterfaceVersion();
  virtual ~InterfaceVersion();

  InterfaceVersion(const InterfaceVersion& from);

  inline InterfaceVersion& operator=(const InterfaceVersion& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  InterfaceVersion(InterfaceVersion&& from) noexcept
    : InterfaceVersion() {
    *this = ::std::move(from);
  }

  inline InterfaceVersion& operator=(InterfaceVersion&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const InterfaceVersion& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const InterfaceVersion* internal_default_instance() {
    return reinterpret_cast<const InterfaceVersion*>(
               &_InterfaceVersion_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(InterfaceVersion* other);
  friend void swap(InterfaceVersion& a, InterfaceVersion& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline InterfaceVersion* New() const PROTOBUF_FINAL { return New(NULL); }

  InterfaceVersion* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const InterfaceVersion& from);
  void MergeFrom(const InterfaceVersion& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(InterfaceVersion* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // uint32 version_major = 1;
  void clear_version_major();
  static const int kVersionMajorFieldNumber = 1;
  ::google::protobuf::uint32 version_major() const;
  void set_version_major(::google::protobuf::uint32 value);

  // uint32 version_minor = 2;
  void clear_version_minor();
  static const int kVersionMinorFieldNumber = 2;
  ::google::protobuf::uint32 version_minor() const;
  void set_version_minor(::google::protobuf::uint32 value);

  // uint32 version_patch = 3;
  void clear_version_patch();
  static const int kVersionPatchFieldNumber = 3;
  ::google::protobuf::uint32 version_patch() const;
  void set_version_patch(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:osi3.InterfaceVersion)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 version_major_;
  ::google::protobuf::uint32 version_minor_;
  ::google::protobuf::uint32 version_patch_;
  mutable int _cached_size_;
  friend struct ::protobuf_osi_5fversion_2eproto::TableStruct;
  friend void ::protobuf_osi_5fversion_2eproto::InitDefaultsInterfaceVersionImpl();
};
// ===================================================================

static const int kCurrentInterfaceVersionFieldNumber = 81000;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::FileOptions,
    ::google::protobuf::internal::MessageTypeTraits< ::osi3::InterfaceVersion >, 11, false >
  current_interface_version;

// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// InterfaceVersion

// uint32 version_major = 1;
inline void InterfaceVersion::clear_version_major() {
  version_major_ = 0u;
}
inline ::google::protobuf::uint32 InterfaceVersion::version_major() const {
  // @@protoc_insertion_point(field_get:osi3.InterfaceVersion.version_major)
  return version_major_;
}
inline void InterfaceVersion::set_version_major(::google::protobuf::uint32 value) {
  
  version_major_ = value;
  // @@protoc_insertion_point(field_set:osi3.InterfaceVersion.version_major)
}

// uint32 version_minor = 2;
inline void InterfaceVersion::clear_version_minor() {
  version_minor_ = 0u;
}
inline ::google::protobuf::uint32 InterfaceVersion::version_minor() const {
  // @@protoc_insertion_point(field_get:osi3.InterfaceVersion.version_minor)
  return version_minor_;
}
inline void InterfaceVersion::set_version_minor(::google::protobuf::uint32 value) {
  
  version_minor_ = value;
  // @@protoc_insertion_point(field_set:osi3.InterfaceVersion.version_minor)
}

// uint32 version_patch = 3;
inline void InterfaceVersion::clear_version_patch() {
  version_patch_ = 0u;
}
inline ::google::protobuf::uint32 InterfaceVersion::version_patch() const {
  // @@protoc_insertion_point(field_get:osi3.InterfaceVersion.version_patch)
  return version_patch_;
}
inline void InterfaceVersion::set_version_patch(::google::protobuf::uint32 value) {
  
  version_patch_ = value;
  // @@protoc_insertion_point(field_set:osi3.InterfaceVersion.version_patch)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace osi3

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_osi_5fversion_2eproto__INCLUDED
