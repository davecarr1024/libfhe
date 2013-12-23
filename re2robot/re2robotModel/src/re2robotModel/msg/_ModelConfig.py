"""autogenerated by genmsg_py from ModelConfig.msg. Do not edit."""
import roslib.message
import struct

import re2robotModel.msg

class ModelConfig(roslib.message.Message):
  _md5sum = "417ae4eb253d437dfccedc437013e3aa"
  _type = "re2robotModel/ModelConfig"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """TransmissionConfig[] transmissions
ArbiterConfig[] arbiters
================================================================================
MSG: re2robotModel/TransmissionConfig
string type
string drive
string joint
float32 timerIntervalS
================================================================================
MSG: re2robotModel/ArbiterConfig
#common
string type
float32 updateIntervalS

#control arbiter
string initialController
"""
  __slots__ = ['transmissions','arbiters']
  _slot_types = ['re2robotModel/TransmissionConfig[]','re2robotModel/ArbiterConfig[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       transmissions,arbiters
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(ModelConfig, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.transmissions is None:
        self.transmissions = []
      if self.arbiters is None:
        self.arbiters = []
    else:
      self.transmissions = []
      self.arbiters = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      length = len(self.transmissions)
      buff.write(_struct_I.pack(length))
      for val1 in self.transmissions:
        _x = val1.type
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.drive
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.joint
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_f.pack(val1.timerIntervalS))
      length = len(self.arbiters)
      buff.write(_struct_I.pack(length))
      for val1 in self.arbiters:
        _x = val1.type
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_f.pack(val1.updateIntervalS))
        _x = val1.initialController
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.transmissions = []
      for i in xrange(0, length):
        val1 = re2robotModel.msg.TransmissionConfig()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.type = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.drive = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.joint = str[start:end]
        start = end
        end += 4
        (val1.timerIntervalS,) = _struct_f.unpack(str[start:end])
        self.transmissions.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.arbiters = []
      for i in xrange(0, length):
        val1 = re2robotModel.msg.ArbiterConfig()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.type = str[start:end]
        start = end
        end += 4
        (val1.updateIntervalS,) = _struct_f.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.initialController = str[start:end]
        self.arbiters.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      length = len(self.transmissions)
      buff.write(_struct_I.pack(length))
      for val1 in self.transmissions:
        _x = val1.type
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.drive
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.joint
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_f.pack(val1.timerIntervalS))
      length = len(self.arbiters)
      buff.write(_struct_I.pack(length))
      for val1 in self.arbiters:
        _x = val1.type
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_f.pack(val1.updateIntervalS))
        _x = val1.initialController
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.transmissions = []
      for i in xrange(0, length):
        val1 = re2robotModel.msg.TransmissionConfig()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.type = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.drive = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.joint = str[start:end]
        start = end
        end += 4
        (val1.timerIntervalS,) = _struct_f.unpack(str[start:end])
        self.transmissions.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.arbiters = []
      for i in xrange(0, length):
        val1 = re2robotModel.msg.ArbiterConfig()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.type = str[start:end]
        start = end
        end += 4
        (val1.updateIntervalS,) = _struct_f.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.initialController = str[start:end]
        self.arbiters.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_f = struct.Struct("<f")