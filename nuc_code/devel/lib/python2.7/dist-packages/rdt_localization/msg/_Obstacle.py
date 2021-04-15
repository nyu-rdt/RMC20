# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rdt_localization/Obstacle.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Obstacle(genpy.Message):
  _md5sum = "8b91b992300f93c3e218076979730b49"
  _type = "rdt_localization/Obstacle"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int64 front_left
int64 front_right
int64 front_left_mid
int64 front_right_mid
int64 depo_front
int64 depo_back"""
  __slots__ = ['front_left','front_right','front_left_mid','front_right_mid','depo_front','depo_back']
  _slot_types = ['int64','int64','int64','int64','int64','int64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       front_left,front_right,front_left_mid,front_right_mid,depo_front,depo_back

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Obstacle, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.front_left is None:
        self.front_left = 0
      if self.front_right is None:
        self.front_right = 0
      if self.front_left_mid is None:
        self.front_left_mid = 0
      if self.front_right_mid is None:
        self.front_right_mid = 0
      if self.depo_front is None:
        self.depo_front = 0
      if self.depo_back is None:
        self.depo_back = 0
    else:
      self.front_left = 0
      self.front_right = 0
      self.front_left_mid = 0
      self.front_right_mid = 0
      self.depo_front = 0
      self.depo_back = 0

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
      buff.write(_get_struct_6q().pack(_x.front_left, _x.front_right, _x.front_left_mid, _x.front_right_mid, _x.depo_front, _x.depo_back))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 48
      (_x.front_left, _x.front_right, _x.front_left_mid, _x.front_right_mid, _x.depo_front, _x.depo_back,) = _get_struct_6q().unpack(str[start:end])
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
      buff.write(_get_struct_6q().pack(_x.front_left, _x.front_right, _x.front_left_mid, _x.front_right_mid, _x.depo_front, _x.depo_back))
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
      end = 0
      _x = self
      start = end
      end += 48
      (_x.front_left, _x.front_right, _x.front_left_mid, _x.front_right_mid, _x.depo_front, _x.depo_back,) = _get_struct_6q().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6q = None
def _get_struct_6q():
    global _struct_6q
    if _struct_6q is None:
        _struct_6q = struct.Struct("<6q")
    return _struct_6q
