// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: osi_detectedoccupant.proto

#ifndef PROTOBUF_osi_5fdetectedoccupant_2eproto__INCLUDED
#define PROTOBUF_osi_5fdetectedoccupant_2eproto__INCLUDED

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
#include "osi_occupant.pb.h"
#include "osi_detectedobject.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_osi_5fdetectedoccupant_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsDetectedOccupant_CandidateOccupantImpl();
void InitDefaultsDetectedOccupant_CandidateOccupant();
void InitDefaultsDetectedOccupantImpl();
void InitDefaultsDetectedOccupant();
inline void InitDefaults() {
  InitDefaultsDetectedOccupant_CandidateOccupant();
  InitDefaultsDetectedOccupant();
}
}  // namespace protobuf_osi_5fdetectedoccupant_2eproto
namespace osi3 {
class DetectedOccupant;
class DetectedOccupantDefaultTypeInternal;
extern DetectedOccupantDefaultTypeInternal _DetectedOccupant_default_instance_;
class DetectedOccupant_CandidateOccupant;
class DetectedOccupant_CandidateOccupantDefaultTypeInternal;
extern DetectedOccupant_CandidateOccupantDefaultTypeInternal _DetectedOccupant_CandidateOccupant_default_instance_;
}  // namespace osi3
namespace osi3 {

// ===================================================================

class DetectedOccupant_CandidateOccupant : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:osi3.DetectedOccupant.CandidateOccupant) */ {
 public:
  DetectedOccupant_CandidateOccupant();
  virtual ~DetectedOccupant_CandidateOccupant();

  DetectedOccupant_CandidateOccupant(const DetectedOccupant_CandidateOccupant& from);

  inline DetectedOccupant_CandidateOccupant& operator=(const DetectedOccupant_CandidateOccupant& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  DetectedOccupant_CandidateOccupant(DetectedOccupant_CandidateOccupant&& from) noexcept
    : DetectedOccupant_CandidateOccupant() {
    *this = ::std::move(from);
  }

  inline DetectedOccupant_CandidateOccupant& operator=(DetectedOccupant_CandidateOccupant&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const DetectedOccupant_CandidateOccupant& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DetectedOccupant_CandidateOccupant* internal_default_instance() {
    return reinterpret_cast<const DetectedOccupant_CandidateOccupant*>(
               &_DetectedOccupant_CandidateOccupant_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(DetectedOccupant_CandidateOccupant* other);
  friend void swap(DetectedOccupant_CandidateOccupant& a, DetectedOccupant_CandidateOccupant& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline DetectedOccupant_CandidateOccupant* New() const PROTOBUF_FINAL { return New(NULL); }

  DetectedOccupant_CandidateOccupant* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const DetectedOccupant_CandidateOccupant& from);
  void MergeFrom(const DetectedOccupant_CandidateOccupant& from);
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
  void InternalSwap(DetectedOccupant_CandidateOccupant* other);
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

  // .osi3.Occupant.Classification classification = 2;
  bool has_classification() const;
  void clear_classification();
  static const int kClassificationFieldNumber = 2;
  const ::osi3::Occupant_Classification& classification() const;
  ::osi3::Occupant_Classification* release_classification();
  ::osi3::Occupant_Classification* mutable_classification();
  void set_allocated_classification(::osi3::Occupant_Classification* classification);

  // double probability = 1;
  void clear_probability();
  static const int kProbabilityFieldNumber = 1;
  double probability() const;
  void set_probability(double value);

  // @@protoc_insertion_point(class_scope:osi3.DetectedOccupant.CandidateOccupant)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::osi3::Occupant_Classification* classification_;
  double probability_;
  mutable int _cached_size_;
  friend struct ::protobuf_osi_5fdetectedoccupant_2eproto::TableStruct;
  friend void ::protobuf_osi_5fdetectedoccupant_2eproto::InitDefaultsDetectedOccupant_CandidateOccupantImpl();
};
// -------------------------------------------------------------------

class DetectedOccupant : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:osi3.DetectedOccupant) */ {
 public:
  DetectedOccupant();
  virtual ~DetectedOccupant();

  DetectedOccupant(const DetectedOccupant& from);

  inline DetectedOccupant& operator=(const DetectedOccupant& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  DetectedOccupant(DetectedOccupant&& from) noexcept
    : DetectedOccupant() {
    *this = ::std::move(from);
  }

  inline DetectedOccupant& operator=(DetectedOccupant&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const DetectedOccupant& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DetectedOccupant* internal_default_instance() {
    return reinterpret_cast<const DetectedOccupant*>(
               &_DetectedOccupant_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(DetectedOccupant* other);
  friend void swap(DetectedOccupant& a, DetectedOccupant& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline DetectedOccupant* New() const PROTOBUF_FINAL { return New(NULL); }

  DetectedOccupant* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const DetectedOccupant& from);
  void MergeFrom(const DetectedOccupant& from);
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
  void InternalSwap(DetectedOccupant* other);
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

  typedef DetectedOccupant_CandidateOccupant CandidateOccupant;

  // accessors -------------------------------------------------------

  // repeated .osi3.DetectedOccupant.CandidateOccupant candidate = 2;
  int candidate_size() const;
  void clear_candidate();
  static const int kCandidateFieldNumber = 2;
  const ::osi3::DetectedOccupant_CandidateOccupant& candidate(int index) const;
  ::osi3::DetectedOccupant_CandidateOccupant* mutable_candidate(int index);
  ::osi3::DetectedOccupant_CandidateOccupant* add_candidate();
  ::google::protobuf::RepeatedPtrField< ::osi3::DetectedOccupant_CandidateOccupant >*
      mutable_candidate();
  const ::google::protobuf::RepeatedPtrField< ::osi3::DetectedOccupant_CandidateOccupant >&
      candidate() const;

  // .osi3.DetectedItemHeader header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::osi3::DetectedItemHeader& header() const;
  ::osi3::DetectedItemHeader* release_header();
  ::osi3::DetectedItemHeader* mutable_header();
  void set_allocated_header(::osi3::DetectedItemHeader* header);

  // @@protoc_insertion_point(class_scope:osi3.DetectedOccupant)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::osi3::DetectedOccupant_CandidateOccupant > candidate_;
  ::osi3::DetectedItemHeader* header_;
  mutable int _cached_size_;
  friend struct ::protobuf_osi_5fdetectedoccupant_2eproto::TableStruct;
  friend void ::protobuf_osi_5fdetectedoccupant_2eproto::InitDefaultsDetectedOccupantImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DetectedOccupant_CandidateOccupant

// double probability = 1;
inline void DetectedOccupant_CandidateOccupant::clear_probability() {
  probability_ = 0;
}
inline double DetectedOccupant_CandidateOccupant::probability() const {
  // @@protoc_insertion_point(field_get:osi3.DetectedOccupant.CandidateOccupant.probability)
  return probability_;
}
inline void DetectedOccupant_CandidateOccupant::set_probability(double value) {
  
  probability_ = value;
  // @@protoc_insertion_point(field_set:osi3.DetectedOccupant.CandidateOccupant.probability)
}

// .osi3.Occupant.Classification classification = 2;
inline bool DetectedOccupant_CandidateOccupant::has_classification() const {
  return this != internal_default_instance() && classification_ != NULL;
}
inline const ::osi3::Occupant_Classification& DetectedOccupant_CandidateOccupant::classification() const {
  const ::osi3::Occupant_Classification* p = classification_;
  // @@protoc_insertion_point(field_get:osi3.DetectedOccupant.CandidateOccupant.classification)
  return p != NULL ? *p : *reinterpret_cast<const ::osi3::Occupant_Classification*>(
      &::osi3::_Occupant_Classification_default_instance_);
}
inline ::osi3::Occupant_Classification* DetectedOccupant_CandidateOccupant::release_classification() {
  // @@protoc_insertion_point(field_release:osi3.DetectedOccupant.CandidateOccupant.classification)
  
  ::osi3::Occupant_Classification* temp = classification_;
  classification_ = NULL;
  return temp;
}
inline ::osi3::Occupant_Classification* DetectedOccupant_CandidateOccupant::mutable_classification() {
  
  if (classification_ == NULL) {
    classification_ = new ::osi3::Occupant_Classification;
  }
  // @@protoc_insertion_point(field_mutable:osi3.DetectedOccupant.CandidateOccupant.classification)
  return classification_;
}
inline void DetectedOccupant_CandidateOccupant::set_allocated_classification(::osi3::Occupant_Classification* classification) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(classification_);
  }
  if (classification) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      classification = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, classification, submessage_arena);
    }
    
  } else {
    
  }
  classification_ = classification;
  // @@protoc_insertion_point(field_set_allocated:osi3.DetectedOccupant.CandidateOccupant.classification)
}

// -------------------------------------------------------------------

// DetectedOccupant

// .osi3.DetectedItemHeader header = 1;
inline bool DetectedOccupant::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::osi3::DetectedItemHeader& DetectedOccupant::header() const {
  const ::osi3::DetectedItemHeader* p = header_;
  // @@protoc_insertion_point(field_get:osi3.DetectedOccupant.header)
  return p != NULL ? *p : *reinterpret_cast<const ::osi3::DetectedItemHeader*>(
      &::osi3::_DetectedItemHeader_default_instance_);
}
inline ::osi3::DetectedItemHeader* DetectedOccupant::release_header() {
  // @@protoc_insertion_point(field_release:osi3.DetectedOccupant.header)
  
  ::osi3::DetectedItemHeader* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::osi3::DetectedItemHeader* DetectedOccupant::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::osi3::DetectedItemHeader;
  }
  // @@protoc_insertion_point(field_mutable:osi3.DetectedOccupant.header)
  return header_;
}
inline void DetectedOccupant::set_allocated_header(::osi3::DetectedItemHeader* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:osi3.DetectedOccupant.header)
}

// repeated .osi3.DetectedOccupant.CandidateOccupant candidate = 2;
inline int DetectedOccupant::candidate_size() const {
  return candidate_.size();
}
inline void DetectedOccupant::clear_candidate() {
  candidate_.Clear();
}
inline const ::osi3::DetectedOccupant_CandidateOccupant& DetectedOccupant::candidate(int index) const {
  // @@protoc_insertion_point(field_get:osi3.DetectedOccupant.candidate)
  return candidate_.Get(index);
}
inline ::osi3::DetectedOccupant_CandidateOccupant* DetectedOccupant::mutable_candidate(int index) {
  // @@protoc_insertion_point(field_mutable:osi3.DetectedOccupant.candidate)
  return candidate_.Mutable(index);
}
inline ::osi3::DetectedOccupant_CandidateOccupant* DetectedOccupant::add_candidate() {
  // @@protoc_insertion_point(field_add:osi3.DetectedOccupant.candidate)
  return candidate_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::osi3::DetectedOccupant_CandidateOccupant >*
DetectedOccupant::mutable_candidate() {
  // @@protoc_insertion_point(field_mutable_list:osi3.DetectedOccupant.candidate)
  return &candidate_;
}
inline const ::google::protobuf::RepeatedPtrField< ::osi3::DetectedOccupant_CandidateOccupant >&
DetectedOccupant::candidate() const {
  // @@protoc_insertion_point(field_list:osi3.DetectedOccupant.candidate)
  return candidate_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace osi3

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_osi_5fdetectedoccupant_2eproto__INCLUDED
