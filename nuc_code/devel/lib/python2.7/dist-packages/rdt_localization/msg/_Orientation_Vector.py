# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rdt_localization/Orientation_Vector.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import rdt_localization.msg

class Orientation_Vector(genpy.Message):
  _md5sum = "09467e8e0e560bedfd1a1c32cc5278c6"
  _type = "rdt_localization/Orientation_Vector"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """Pose robot_pose
Location dig_zone
int64 robot_speed
================================================================================
MSG: rdt_localization/Pose
float32 x
float32 y
float32 orientation
================================================================================
MSG: rdt_localization/Location
float32 x
float32 y"""
  __slots__ = ['robot_pose','dig_zone','robot_speed']
  _slot_types = ['rdt_localization/Pose','rdt_localization/Location','int64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       robot_pose,dig_zone,robot_speed

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Orientation_Vector, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.robot_pose is None:
        self.robot_pose = rdt_localization.msg.Pose()
      if self.dig_zone is None:
        self.dig_zone = rdt_localization.msg.Location()
      if self.robot_speed is None:
        self.robot_speed = 0
    else:
      self.robot_pose = rdt_localization.msg.Pose()
      self.dig_zone = rdt_localization.msg.Location()
      self.robot_speed = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_5fq().pack(_x.robot_pose.x, _x.robot_pose.y, _x.robot_pose.orientation, _x.dig_zone.x, _x.dig_zone.y, _x.robot_speed))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.robot_pose is None:
        self.robot_pose = rdt_localization.msg.Pose()
      if self.dig_zone is None:
        self.dig_zone = rdt_localization.msg.Location()
      end = 0
      _x = self
      start = end
      end += 28
      (_x.robot_pose.x, _x.robot_pose.y, _x.robot_pose.orientation, _x.dig_zone.x, _x.dig_zone.y, _x.robot_speed,) = _get_struct_5fq().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_5fq().pack(_x.robot_pose.x, _x.robot_pose.y, _x.robot_pose.orientation, _x.dig_zone.x, _x.dig_zone.y, _x.robot_speed))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.robot_pose is None:
        self.robot_pose = rdt_localization.msg.Pose()
      if self.dig_zone is None:
        self.dig_zone = rdt_localization.msg.Location()
      end = 0
      _x = self
      start = end
      end += 28
      (_x.robot_pose.x, _x.robot_pose.y, _x.robot_pose.orientation, _x.dig_zone.x, _x.dig_zone.y, _x.robot_speed,) = _get_struct_5fq().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5fq = None
def _get_struct_5fq():
    global _struct_5fq
    if _struct_5fq is None:
        _struct_5fq = struct.Struct("<5fq")
    return _struct_5fq
