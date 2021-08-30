// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: osi_datarecording.proto

#include "osi_datarecording.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)
namespace osi3 {
class SensorDataSeriesDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<SensorDataSeries>
      _instance;
} _SensorDataSeries_default_instance_;
class SensorDataSeriesListDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<SensorDataSeriesList>
      _instance;
} _SensorDataSeriesList_default_instance_;
}  // namespace osi3
namespace protobuf_osi_5fdatarecording_2eproto {
void InitDefaultsSensorDataSeriesImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  protobuf_osi_5fsensordata_2eproto::InitDefaultsSensorData();
  {
    void* ptr = &::osi3::_SensorDataSeries_default_instance_;
    new (ptr) ::osi3::SensorDataSeries();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::osi3::SensorDataSeries::InitAsDefaultInstance();
}

void InitDefaultsSensorDataSeries() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsSensorDataSeriesImpl);
}

void InitDefaultsSensorDataSeriesListImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  protobuf_osi_5fdatarecording_2eproto::InitDefaultsSensorDataSeries();
  {
    void* ptr = &::osi3::_SensorDataSeriesList_default_instance_;
    new (ptr) ::osi3::SensorDataSeriesList();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::osi3::SensorDataSeriesList::InitAsDefaultInstance();
}

void InitDefaultsSensorDataSeriesList() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsSensorDataSeriesListImpl);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::osi3::SensorDataSeries, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::osi3::SensorDataSeries, sensor_data_),
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::osi3::SensorDataSeriesList, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::osi3::SensorDataSeriesList, sensor_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::osi3::SensorDataSeries)},
  { 6, -1, sizeof(::osi3::SensorDataSeriesList)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::osi3::_SensorDataSeries_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::osi3::_SensorDataSeriesList_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "osi_datarecording.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\027osi_datarecording.proto\022\004osi3\032\024osi_sen"
      "sordata.proto\"9\n\020SensorDataSeries\022%\n\013sen"
      "sor_data\030\001 \003(\0132\020.osi3.SensorData\">\n\024Sens"
      "orDataSeriesList\022&\n\006sensor\030\001 \003(\0132\026.osi3."
      "SensorDataSeriesB\002H\001b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 188);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "osi_datarecording.proto", &protobuf_RegisterTypes);
  ::protobuf_osi_5fsensordata_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_osi_5fdatarecording_2eproto
namespace osi3 {

// ===================================================================

void SensorDataSeries::InitAsDefaultInstance() {
}
void SensorDataSeries::clear_sensor_data() {
  sensor_data_.Clear();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int SensorDataSeries::kSensorDataFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

SensorDataSeries::SensorDataSeries()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_osi_5fdatarecording_2eproto::InitDefaultsSensorDataSeries();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:osi3.SensorDataSeries)
}
SensorDataSeries::SensorDataSeries(const SensorDataSeries& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      sensor_data_(from.sensor_data_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:osi3.SensorDataSeries)
}

void SensorDataSeries::SharedCtor() {
  _cached_size_ = 0;
}

SensorDataSeries::~SensorDataSeries() {
  // @@protoc_insertion_point(destructor:osi3.SensorDataSeries)
  SharedDtor();
}

void SensorDataSeries::SharedDtor() {
}

void SensorDataSeries::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* SensorDataSeries::descriptor() {
  ::protobuf_osi_5fdatarecording_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_osi_5fdatarecording_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const SensorDataSeries& SensorDataSeries::default_instance() {
  ::protobuf_osi_5fdatarecording_2eproto::InitDefaultsSensorDataSeries();
  return *internal_default_instance();
}

SensorDataSeries* SensorDataSeries::New(::google::protobuf::Arena* arena) const {
  SensorDataSeries* n = new SensorDataSeries;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void SensorDataSeries::Clear() {
// @@protoc_insertion_point(message_clear_start:osi3.SensorDataSeries)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  sensor_data_.Clear();
  _internal_metadata_.Clear();
}

bool SensorDataSeries::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:osi3.SensorDataSeries)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .osi3.SensorData sensor_data = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(input, add_sensor_data()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:osi3.SensorDataSeries)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:osi3.SensorDataSeries)
  return false;
#undef DO_
}

void SensorDataSeries::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:osi3.SensorDataSeries)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .osi3.SensorData sensor_data = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->sensor_data_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->sensor_data(static_cast<int>(i)), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:osi3.SensorDataSeries)
}

::google::protobuf::uint8* SensorDataSeries::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:osi3.SensorDataSeries)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .osi3.SensorData sensor_data = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->sensor_data_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->sensor_data(static_cast<int>(i)), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:osi3.SensorDataSeries)
  return target;
}

size_t SensorDataSeries::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:osi3.SensorDataSeries)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated .osi3.SensorData sensor_data = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->sensor_data_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->sensor_data(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void SensorDataSeries::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:osi3.SensorDataSeries)
  GOOGLE_DCHECK_NE(&from, this);
  const SensorDataSeries* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const SensorDataSeries>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:osi3.SensorDataSeries)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:osi3.SensorDataSeries)
    MergeFrom(*source);
  }
}

void SensorDataSeries::MergeFrom(const SensorDataSeries& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:osi3.SensorDataSeries)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  sensor_data_.MergeFrom(from.sensor_data_);
}

void SensorDataSeries::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:osi3.SensorDataSeries)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SensorDataSeries::CopyFrom(const SensorDataSeries& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:osi3.SensorDataSeries)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SensorDataSeries::IsInitialized() const {
  return true;
}

void SensorDataSeries::Swap(SensorDataSeries* other) {
  if (other == this) return;
  InternalSwap(other);
}
void SensorDataSeries::InternalSwap(SensorDataSeries* other) {
  using std::swap;
  sensor_data_.InternalSwap(&other->sensor_data_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata SensorDataSeries::GetMetadata() const {
  protobuf_osi_5fdatarecording_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_osi_5fdatarecording_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void SensorDataSeriesList::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int SensorDataSeriesList::kSensorFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

SensorDataSeriesList::SensorDataSeriesList()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_osi_5fdatarecording_2eproto::InitDefaultsSensorDataSeriesList();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:osi3.SensorDataSeriesList)
}
SensorDataSeriesList::SensorDataSeriesList(const SensorDataSeriesList& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      sensor_(from.sensor_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:osi3.SensorDataSeriesList)
}

void SensorDataSeriesList::SharedCtor() {
  _cached_size_ = 0;
}

SensorDataSeriesList::~SensorDataSeriesList() {
  // @@protoc_insertion_point(destructor:osi3.SensorDataSeriesList)
  SharedDtor();
}

void SensorDataSeriesList::SharedDtor() {
}

void SensorDataSeriesList::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* SensorDataSeriesList::descriptor() {
  ::protobuf_osi_5fdatarecording_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_osi_5fdatarecording_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const SensorDataSeriesList& SensorDataSeriesList::default_instance() {
  ::protobuf_osi_5fdatarecording_2eproto::InitDefaultsSensorDataSeriesList();
  return *internal_default_instance();
}

SensorDataSeriesList* SensorDataSeriesList::New(::google::protobuf::Arena* arena) const {
  SensorDataSeriesList* n = new SensorDataSeriesList;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void SensorDataSeriesList::Clear() {
// @@protoc_insertion_point(message_clear_start:osi3.SensorDataSeriesList)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  sensor_.Clear();
  _internal_metadata_.Clear();
}

bool SensorDataSeriesList::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:osi3.SensorDataSeriesList)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .osi3.SensorDataSeries sensor = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(input, add_sensor()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:osi3.SensorDataSeriesList)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:osi3.SensorDataSeriesList)
  return false;
#undef DO_
}

void SensorDataSeriesList::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:osi3.SensorDataSeriesList)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .osi3.SensorDataSeries sensor = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->sensor_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->sensor(static_cast<int>(i)), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:osi3.SensorDataSeriesList)
}

::google::protobuf::uint8* SensorDataSeriesList::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:osi3.SensorDataSeriesList)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .osi3.SensorDataSeries sensor = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->sensor_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->sensor(static_cast<int>(i)), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:osi3.SensorDataSeriesList)
  return target;
}

size_t SensorDataSeriesList::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:osi3.SensorDataSeriesList)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated .osi3.SensorDataSeries sensor = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->sensor_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->sensor(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void SensorDataSeriesList::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:osi3.SensorDataSeriesList)
  GOOGLE_DCHECK_NE(&from, this);
  const SensorDataSeriesList* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const SensorDataSeriesList>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:osi3.SensorDataSeriesList)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:osi3.SensorDataSeriesList)
    MergeFrom(*source);
  }
}

void SensorDataSeriesList::MergeFrom(const SensorDataSeriesList& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:osi3.SensorDataSeriesList)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  sensor_.MergeFrom(from.sensor_);
}

void SensorDataSeriesList::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:osi3.SensorDataSeriesList)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SensorDataSeriesList::CopyFrom(const SensorDataSeriesList& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:osi3.SensorDataSeriesList)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SensorDataSeriesList::IsInitialized() const {
  return true;
}

void SensorDataSeriesList::Swap(SensorDataSeriesList* other) {
  if (other == this) return;
  InternalSwap(other);
}
void SensorDataSeriesList::InternalSwap(SensorDataSeriesList* other) {
  using std::swap;
  sensor_.InternalSwap(&other->sensor_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata SensorDataSeriesList::GetMetadata() const {
  protobuf_osi_5fdatarecording_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_osi_5fdatarecording_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace osi3

// @@protoc_insertion_point(global_scope)