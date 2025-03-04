# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: AR-OS.proto
# Protobuf Python Version: 5.29.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import runtime_version as _runtime_version
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
_runtime_version.ValidateProtobufRuntimeVersion(
    _runtime_version.Domain.PUBLIC,
    5,
    29,
    1,
    '',
    'AR-OS.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0b\x41R-OS.proto\">\n\x0c\x41ROS_Command\x12\x19\n\x07\x63ommand\x18\x01 \x02(\x0e\x32\x08.COMMAND\x12\x13\n\x0b\x62yte_string\x18\x02 \x01(\x0c\"o\n\x12Simulator_Response\x12\x1b\n\x08response\x18\x01 \x02(\x0e\x32\t.RESPONSE\x12\x0e\n\x06single\x18\x02 \x01(\x02\x12\x17\n\x06vector\x18\x03 \x01(\x0b\x32\x07.VECTOR\x12\x13\n\x0b\x62yte_string\x18\x04 \x01(\x0c\")\n\x06VECTOR\x12\t\n\x01x\x18\x01 \x02(\x02\x12\t\n\x01y\x18\x02 \x02(\x02\x12\t\n\x01z\x18\x03 \x02(\x02*\xee\x04\n\x07\x43OMMAND\x12\x0c\n\x08GEN_PING\x10\x01\x12\x13\n\x0fGEN_GET_VOLTAGE\x10\x03\x12\x10\n\x0cGEN_GET_TEMP\x10\x04\x12\x12\n\x0e\x45PS_GET_CHARGE\x10\x05\x12\x0e\n\nEPS_GET_PS\x10\x1d\x12\x11\n\rEPS_SET_PS_ON\x10\x1e\x12\x12\n\x0e\x45PS_SET_PS_OFF\x10\x1f\x12\x10\n\x0c\x45SP_GET_FUEL\x10\x06\x12\x10\n\x0c\x45SP_GET_MODE\x10\x07\x12\x12\n\x0e\x45SP_SET_WARMUP\x10\x08\x12\x13\n\x0f\x45SP_SET_BURNING\x10\t\x12\x0f\n\x0b\x45SP_SET_OFF\x10\n\x12\x11\n\rDRAG_GET_MODE\x10\x0b\x12\x13\n\x0f\x44RAG_SET_DEPLOY\x10\x0c\x12\x10\n\x0c\x41\x44\x43S_GET_PRY\x10\r\x12\x11\n\rADCS_GET_MODE\x10\x0e\x12\x10\n\x0c\x41\x44\x43S_SET_OFF\x10\x0f\x12\x16\n\x12\x41\x44\x43S_SET_DE_TUMBLE\x10\x10\x12\x16\n\x12\x41\x44\x43S_SET_SUN_POINT\x10\x11\x12\x11\n\rGNSS_GET_POSI\x10\x12\x12\x0f\n\x0bPI_GET_MODE\x10\x13\x12\x10\n\x0cPI_GET_AUDIO\x10\x14\x12\r\n\tPI_SET_ON\x10\x15\x12\x0e\n\nPI_SET_OFF\x10\x16\x12\x10\n\x0cTTC_GET_MODE\x10\x17\x12\x17\n\x13TTC_GET_BYTE_STRING\x10\x18\x12\x0f\n\x0bTTC_SET_OFF\x10\x19\x12\x15\n\x11TTC_SET_BEACONING\x10\x1a\x12\x16\n\x12TTC_SET_CONNECTING\x10\x1b\x12\x1c\n\x18TTC_SET_BROADCAST_NO_CON\x10 \x12\x18\n\x14TTC_SEND_BYTE_STRING\x10\x1c*\x92\x04\n\x08RESPONSE\x12\x0c\n\x08GEN_PONG\x10\x01\x12\r\n\tGEN_ERROR\x10\x02\x12\x0f\n\x0bGEN_SUCCESS\x10\x17\x12\x15\n\x11GEN_RETURN_SINGLE\x10\x03\x12\x15\n\x11GEM_RETURN_VECTOR\x10\x04\x12\x1a\n\x16GEN_RETURN_BYTE_STRING\x10\x10\x12\r\n\tEPS_PS_ON\x10\x18\x12\x0e\n\nEPS_PS_OFF\x10\x19\x12\x0b\n\x07\x45SP_OFF\x10\x05\x12\x0f\n\x0b\x45SP_WARMING\x10\x06\x12\r\n\tESP_READY\x10\x07\x12\x0f\n\x0b\x45SP_BURNING\x10\t\x12\x11\n\rESP_COOL_DOWN\x10\n\x12\x12\n\x0e\x44RAG_RETRACTED\x10\x0b\x12\x11\n\rDRAG_DEPLOYED\x10\x0c\x12\x0c\n\x08\x41\x44\x43S_OFF\x10\r\x12\x12\n\x0e\x41\x44\x43S_DE_TUMBLE\x10\x0e\x12\x12\n\x0e\x41\x44\x43S_SUN_POINT\x10\x0f\x12\t\n\x05PI_ON\x10\x11\x12\n\n\x06PI_OFF\x10\x12\x12\x0b\n\x07TTC_OFF\x10\x13\x12\x11\n\rTTC_BEACONING\x10\x14\x12\x12\n\x0eTTC_CONNECTING\x10\x15\x12\x18\n\x14TTC_ESTABLISHED_DATA\x10\x16\x12\x18\n\x14TTC_ESTABLISHED_CONT\x10\x1a\x12\x18\n\x14TTC_BROADCAST_NO_CON\x10\x1b\x12\x14\n\x10TTC_DISCONNECTED\x10\x1c\x12\x12\n\x0eTTC_GS_COMMAND\x10\x1d')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'AR_OS_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_COMMAND']._serialized_start=236
  _globals['_COMMAND']._serialized_end=858
  _globals['_RESPONSE']._serialized_start=861
  _globals['_RESPONSE']._serialized_end=1391
  _globals['_AROS_COMMAND']._serialized_start=15
  _globals['_AROS_COMMAND']._serialized_end=77
  _globals['_SIMULATOR_RESPONSE']._serialized_start=79
  _globals['_SIMULATOR_RESPONSE']._serialized_end=190
  _globals['_VECTOR']._serialized_start=192
  _globals['_VECTOR']._serialized_end=233
# @@protoc_insertion_point(module_scope)
