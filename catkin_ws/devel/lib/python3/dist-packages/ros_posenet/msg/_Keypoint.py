# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ros_posenet/Keypoint.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Keypoint(genpy.Message):
  _md5sum = "ed7553fef5f9989af54ac621aed181dd"
  _type = "ros_posenet/Keypoint"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """string part
float64 score
geometry_msgs/Point position
geometry_msgs/Point image_position

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['part','score','position','image_position']
  _slot_types = ['string','float64','geometry_msgs/Point','geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       part,score,position,image_position

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Keypoint, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.part is None:
        self.part = ''
      if self.score is None:
        self.score = 0.
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.image_position is None:
        self.image_position = geometry_msgs.msg.Point()
    else:
      self.part = ''
      self.score = 0.
      self.position = geometry_msgs.msg.Point()
      self.image_position = geometry_msgs.msg.Point()

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
      _x = self.part
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.score, _x.position.x, _x.position.y, _x.position.z, _x.image_position.x, _x.image_position.y, _x.image_position.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.image_position is None:
        self.image_position = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.part = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.part = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.score, _x.position.x, _x.position.y, _x.position.z, _x.image_position.x, _x.image_position.y, _x.image_position.z,) = _get_struct_7d().unpack(str[start:end])
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
      _x = self.part
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.score, _x.position.x, _x.position.y, _x.position.z, _x.image_position.x, _x.image_position.y, _x.image_position.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.image_position is None:
        self.image_position = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.part = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.part = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.score, _x.position.x, _x.position.y, _x.position.z, _x.image_position.x, _x.image_position.y, _x.image_position.z,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d