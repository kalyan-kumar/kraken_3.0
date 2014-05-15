"""autogenerated by genpy from kraken_msgs/controllerResult.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class controllerResult(genpy.Message):
  _md5sum = "c4ed7083844e87c15cbd06b69847e794"
  _type = "kraken_msgs/controllerResult"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#result
float32 heading_final
float32 forward_final
float32 rightmove_final
float32 depth_final



"""
  __slots__ = ['heading_final','forward_final','rightmove_final','depth_final']
  _slot_types = ['float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       heading_final,forward_final,rightmove_final,depth_final

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(controllerResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.heading_final is None:
        self.heading_final = 0.
      if self.forward_final is None:
        self.forward_final = 0.
      if self.rightmove_final is None:
        self.rightmove_final = 0.
      if self.depth_final is None:
        self.depth_final = 0.
    else:
      self.heading_final = 0.
      self.forward_final = 0.
      self.rightmove_final = 0.
      self.depth_final = 0.

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
      buff.write(_struct_4f.pack(_x.heading_final, _x.forward_final, _x.rightmove_final, _x.depth_final))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.heading_final, _x.forward_final, _x.rightmove_final, _x.depth_final,) = _struct_4f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_4f.pack(_x.heading_final, _x.forward_final, _x.rightmove_final, _x.depth_final))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.heading_final, _x.forward_final, _x.rightmove_final, _x.depth_final,) = _struct_4f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4f = struct.Struct("<4f")
