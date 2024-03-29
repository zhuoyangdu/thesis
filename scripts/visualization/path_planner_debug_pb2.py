# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: path_planner_debug.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='path_planner_debug.proto',
  package='planning.debug',
  syntax='proto2',
  serialized_pb=_b('\n\x18path_planner_debug.proto\x12\x0eplanning.debug\"\xe2\x02\n\x10PathPlannerDebug\x12\"\n\x04tree\x18\x01 \x01(\x0b\x32\x14.planning.debug.Tree\x12\"\n\x04path\x18\x02 \x01(\x0b\x32\x14.planning.debug.Path\x12)\n\x0bspline_path\x18\x03 \x01(\x0b\x32\x14.planning.debug.Path\x12/\n\x0bglobal_path\x18\x04 \x01(\x0b\x32\x1a.planning.debug.GlobalPath\x12\x46\n\x17obstacle_image_polygons\x18\x05 \x01(\x0b\x32%.planning.debug.ObstacleImagePolygons\x12\x32\n\rvehicle_state\x18\x06 \x01(\x0b\x32\x1b.planning.debug.GlobalPoint\x12.\n\tobstacles\x18\x07 \x03(\x0b\x32\x1b.planning.debug.GlobalPoint\"1\n\x04Tree\x12)\n\x05nodes\x18\x01 \x03(\x0b\x32\x1a.planning.debug.ImagePoint\"K\n\nImagePoint\x12\r\n\x05index\x18\x01 \x01(\x05\x12\x14\n\x0cparent_index\x18\x02 \x01(\x05\x12\x0b\n\x03row\x18\x03 \x01(\x01\x12\x0b\n\x03\x63ol\x18\x04 \x01(\x01\"1\n\x04Path\x12)\n\x05nodes\x18\x01 \x03(\x0b\x32\x1a.planning.debug.ImagePoint\"=\n\x0bGlobalPoint\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\r\n\x05theta\x18\x03 \x01(\x01\x12\t\n\x01s\x18\x04 \x01(\x01\"9\n\nGlobalPath\x12+\n\x06points\x18\x01 \x03(\x0b\x32\x1b.planning.debug.GlobalPoint\"P\n\x15ObstacleImagePolygons\x12\x37\n\tobstacles\x18\x01 \x03(\x0b\x32$.planning.debug.ObstacleImagePolygon\"D\n\x14ObstacleImagePolygon\x12,\n\x08vertexes\x18\x01 \x03(\x0b\x32\x1a.planning.debug.ImagePoint')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_PATHPLANNERDEBUG = _descriptor.Descriptor(
  name='PathPlannerDebug',
  full_name='planning.debug.PathPlannerDebug',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='tree', full_name='planning.debug.PathPlannerDebug.tree', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='path', full_name='planning.debug.PathPlannerDebug.path', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='spline_path', full_name='planning.debug.PathPlannerDebug.spline_path', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='global_path', full_name='planning.debug.PathPlannerDebug.global_path', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='obstacle_image_polygons', full_name='planning.debug.PathPlannerDebug.obstacle_image_polygons', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vehicle_state', full_name='planning.debug.PathPlannerDebug.vehicle_state', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='obstacles', full_name='planning.debug.PathPlannerDebug.obstacles', index=6,
      number=7, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=45,
  serialized_end=399,
)


_TREE = _descriptor.Descriptor(
  name='Tree',
  full_name='planning.debug.Tree',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='nodes', full_name='planning.debug.Tree.nodes', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=401,
  serialized_end=450,
)


_IMAGEPOINT = _descriptor.Descriptor(
  name='ImagePoint',
  full_name='planning.debug.ImagePoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='index', full_name='planning.debug.ImagePoint.index', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='parent_index', full_name='planning.debug.ImagePoint.parent_index', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='row', full_name='planning.debug.ImagePoint.row', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='col', full_name='planning.debug.ImagePoint.col', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=452,
  serialized_end=527,
)


_PATH = _descriptor.Descriptor(
  name='Path',
  full_name='planning.debug.Path',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='nodes', full_name='planning.debug.Path.nodes', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=529,
  serialized_end=578,
)


_GLOBALPOINT = _descriptor.Descriptor(
  name='GlobalPoint',
  full_name='planning.debug.GlobalPoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='planning.debug.GlobalPoint.x', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='planning.debug.GlobalPoint.y', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='theta', full_name='planning.debug.GlobalPoint.theta', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s', full_name='planning.debug.GlobalPoint.s', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=580,
  serialized_end=641,
)


_GLOBALPATH = _descriptor.Descriptor(
  name='GlobalPath',
  full_name='planning.debug.GlobalPath',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='points', full_name='planning.debug.GlobalPath.points', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=643,
  serialized_end=700,
)


_OBSTACLEIMAGEPOLYGONS = _descriptor.Descriptor(
  name='ObstacleImagePolygons',
  full_name='planning.debug.ObstacleImagePolygons',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='obstacles', full_name='planning.debug.ObstacleImagePolygons.obstacles', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=702,
  serialized_end=782,
)


_OBSTACLEIMAGEPOLYGON = _descriptor.Descriptor(
  name='ObstacleImagePolygon',
  full_name='planning.debug.ObstacleImagePolygon',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vertexes', full_name='planning.debug.ObstacleImagePolygon.vertexes', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=784,
  serialized_end=852,
)

_PATHPLANNERDEBUG.fields_by_name['tree'].message_type = _TREE
_PATHPLANNERDEBUG.fields_by_name['path'].message_type = _PATH
_PATHPLANNERDEBUG.fields_by_name['spline_path'].message_type = _PATH
_PATHPLANNERDEBUG.fields_by_name['global_path'].message_type = _GLOBALPATH
_PATHPLANNERDEBUG.fields_by_name['obstacle_image_polygons'].message_type = _OBSTACLEIMAGEPOLYGONS
_PATHPLANNERDEBUG.fields_by_name['vehicle_state'].message_type = _GLOBALPOINT
_PATHPLANNERDEBUG.fields_by_name['obstacles'].message_type = _GLOBALPOINT
_TREE.fields_by_name['nodes'].message_type = _IMAGEPOINT
_PATH.fields_by_name['nodes'].message_type = _IMAGEPOINT
_GLOBALPATH.fields_by_name['points'].message_type = _GLOBALPOINT
_OBSTACLEIMAGEPOLYGONS.fields_by_name['obstacles'].message_type = _OBSTACLEIMAGEPOLYGON
_OBSTACLEIMAGEPOLYGON.fields_by_name['vertexes'].message_type = _IMAGEPOINT
DESCRIPTOR.message_types_by_name['PathPlannerDebug'] = _PATHPLANNERDEBUG
DESCRIPTOR.message_types_by_name['Tree'] = _TREE
DESCRIPTOR.message_types_by_name['ImagePoint'] = _IMAGEPOINT
DESCRIPTOR.message_types_by_name['Path'] = _PATH
DESCRIPTOR.message_types_by_name['GlobalPoint'] = _GLOBALPOINT
DESCRIPTOR.message_types_by_name['GlobalPath'] = _GLOBALPATH
DESCRIPTOR.message_types_by_name['ObstacleImagePolygons'] = _OBSTACLEIMAGEPOLYGONS
DESCRIPTOR.message_types_by_name['ObstacleImagePolygon'] = _OBSTACLEIMAGEPOLYGON

PathPlannerDebug = _reflection.GeneratedProtocolMessageType('PathPlannerDebug', (_message.Message,), dict(
  DESCRIPTOR = _PATHPLANNERDEBUG,
  __module__ = 'path_planner_debug_pb2'
  # @@protoc_insertion_point(class_scope:planning.debug.PathPlannerDebug)
  ))
_sym_db.RegisterMessage(PathPlannerDebug)

Tree = _reflection.GeneratedProtocolMessageType('Tree', (_message.Message,), dict(
  DESCRIPTOR = _TREE,
  __module__ = 'path_planner_debug_pb2'
  # @@protoc_insertion_point(class_scope:planning.debug.Tree)
  ))
_sym_db.RegisterMessage(Tree)

ImagePoint = _reflection.GeneratedProtocolMessageType('ImagePoint', (_message.Message,), dict(
  DESCRIPTOR = _IMAGEPOINT,
  __module__ = 'path_planner_debug_pb2'
  # @@protoc_insertion_point(class_scope:planning.debug.ImagePoint)
  ))
_sym_db.RegisterMessage(ImagePoint)

Path = _reflection.GeneratedProtocolMessageType('Path', (_message.Message,), dict(
  DESCRIPTOR = _PATH,
  __module__ = 'path_planner_debug_pb2'
  # @@protoc_insertion_point(class_scope:planning.debug.Path)
  ))
_sym_db.RegisterMessage(Path)

GlobalPoint = _reflection.GeneratedProtocolMessageType('GlobalPoint', (_message.Message,), dict(
  DESCRIPTOR = _GLOBALPOINT,
  __module__ = 'path_planner_debug_pb2'
  # @@protoc_insertion_point(class_scope:planning.debug.GlobalPoint)
  ))
_sym_db.RegisterMessage(GlobalPoint)

GlobalPath = _reflection.GeneratedProtocolMessageType('GlobalPath', (_message.Message,), dict(
  DESCRIPTOR = _GLOBALPATH,
  __module__ = 'path_planner_debug_pb2'
  # @@protoc_insertion_point(class_scope:planning.debug.GlobalPath)
  ))
_sym_db.RegisterMessage(GlobalPath)

ObstacleImagePolygons = _reflection.GeneratedProtocolMessageType('ObstacleImagePolygons', (_message.Message,), dict(
  DESCRIPTOR = _OBSTACLEIMAGEPOLYGONS,
  __module__ = 'path_planner_debug_pb2'
  # @@protoc_insertion_point(class_scope:planning.debug.ObstacleImagePolygons)
  ))
_sym_db.RegisterMessage(ObstacleImagePolygons)

ObstacleImagePolygon = _reflection.GeneratedProtocolMessageType('ObstacleImagePolygon', (_message.Message,), dict(
  DESCRIPTOR = _OBSTACLEIMAGEPOLYGON,
  __module__ = 'path_planner_debug_pb2'
  # @@protoc_insertion_point(class_scope:planning.debug.ObstacleImagePolygon)
  ))
_sym_db.RegisterMessage(ObstacleImagePolygon)


# @@protoc_insertion_point(module_scope)
