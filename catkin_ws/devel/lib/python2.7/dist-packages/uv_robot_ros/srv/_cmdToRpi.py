# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from uv_robot_ros/cmdToRpiRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class cmdToRpiRequest(genpy.Message):
  _md5sum = "5b12a6073d39992539a74c9b99e2cd41"
  _type = "uv_robot_ros/cmdToRpiRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """string cmdType
float32 dist_or_deg
"""
  __slots__ = ['cmdType','dist_or_deg']
  _slot_types = ['string','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       cmdType,dist_or_deg

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(cmdToRpiRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.cmdType is None:
        self.cmdType = ''
      if self.dist_or_deg is None:
        self.dist_or_deg = 0.
    else:
      self.cmdType = ''
      self.dist_or_deg = 0.

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
      _x = self.cmdType
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.dist_or_deg
      buff.write(_get_struct_f().pack(_x))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmdType = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.cmdType = str[start:end]
      start = end
      end += 4
      (self.dist_or_deg,) = _get_struct_f().unpack(str[start:end])
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
      _x = self.cmdType
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.dist_or_deg
      buff.write(_get_struct_f().pack(_x))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmdType = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.cmdType = str[start:end]
      start = end
      end += 4
      (self.dist_or_deg,) = _get_struct_f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from uv_robot_ros/cmdToRpiResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class cmdToRpiResponse(genpy.Message):
  _md5sum = "d6ac61039326de273907480d4c529a6e"
  _type = "uv_robot_ros/cmdToRpiResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool isComplete
string errorMsg

"""
  __slots__ = ['isComplete','errorMsg']
  _slot_types = ['bool','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       isComplete,errorMsg

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(cmdToRpiResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.isComplete is None:
        self.isComplete = False
      if self.errorMsg is None:
        self.errorMsg = ''
    else:
      self.isComplete = False
      self.errorMsg = ''

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
      _x = self.isComplete
      buff.write(_get_struct_B().pack(_x))
      _x = self.errorMsg
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      start = end
      end += 1
      (self.isComplete,) = _get_struct_B().unpack(str[start:end])
      self.isComplete = bool(self.isComplete)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.errorMsg = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.errorMsg = str[start:end]
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
      _x = self.isComplete
      buff.write(_get_struct_B().pack(_x))
      _x = self.errorMsg
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      start = end
      end += 1
      (self.isComplete,) = _get_struct_B().unpack(str[start:end])
      self.isComplete = bool(self.isComplete)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.errorMsg = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.errorMsg = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class cmdToRpi(object):
  _type          = 'uv_robot_ros/cmdToRpi'
  _md5sum = '4b2b21c7747f71f43af6626178a5bf39'
  _request_class  = cmdToRpiRequest
  _response_class = cmdToRpiResponse