"""autogenerated by genmsg_py from DriveConfig.msg. Do not edit."""
import roslib.message
import struct


class DriveConfig(roslib.message.Message):
  _md5sum = "7739876033e4c5d57efaa789aba3f14f"
  _type = "re2robotDriver/DriveConfig"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#Drive
string name
string type
float32 updateIntervalS
float32 cmdTimeoutS
float32 maxVelocity
float32 minPosition
float32 maxPosition

#SimDrive
float32 initialPosition
float32 positionPGain
float32 positionDGain
float32 velocityTimeToTargetS
"""
  __slots__ = ['name','type','updateIntervalS','cmdTimeoutS','maxVelocity','minPosition','maxPosition','initialPosition','positionPGain','positionDGain','velocityTimeToTargetS']
  _slot_types = ['string','string','float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       name,type,updateIntervalS,cmdTimeoutS,maxVelocity,minPosition,maxPosition,initialPosition,positionPGain,positionDGain,velocityTimeToTargetS
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(DriveConfig, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.name is None:
        self.name = ''
      if self.type is None:
        self.type = ''
      if self.updateIntervalS is None:
        self.updateIntervalS = 0.
      if self.cmdTimeoutS is None:
        self.cmdTimeoutS = 0.
      if self.maxVelocity is None:
        self.maxVelocity = 0.
      if self.minPosition is None:
        self.minPosition = 0.
      if self.maxPosition is None:
        self.maxPosition = 0.
      if self.initialPosition is None:
        self.initialPosition = 0.
      if self.positionPGain is None:
        self.positionPGain = 0.
      if self.positionDGain is None:
        self.positionDGain = 0.
      if self.velocityTimeToTargetS is None:
        self.velocityTimeToTargetS = 0.
    else:
      self.name = ''
      self.type = ''
      self.updateIntervalS = 0.
      self.cmdTimeoutS = 0.
      self.maxVelocity = 0.
      self.minPosition = 0.
      self.maxPosition = 0.
      self.initialPosition = 0.
      self.positionPGain = 0.
      self.positionDGain = 0.
      self.velocityTimeToTargetS = 0.

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
      _x = self.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.type
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_9f.pack(_x.updateIntervalS, _x.cmdTimeoutS, _x.maxVelocity, _x.minPosition, _x.maxPosition, _x.initialPosition, _x.positionPGain, _x.positionDGain, _x.velocityTimeToTargetS))
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
      start = end
      end += length
      self.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.type = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.updateIntervalS, _x.cmdTimeoutS, _x.maxVelocity, _x.minPosition, _x.maxPosition, _x.initialPosition, _x.positionPGain, _x.positionDGain, _x.velocityTimeToTargetS,) = _struct_9f.unpack(str[start:end])
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
      _x = self.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.type
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_9f.pack(_x.updateIntervalS, _x.cmdTimeoutS, _x.maxVelocity, _x.minPosition, _x.maxPosition, _x.initialPosition, _x.positionPGain, _x.positionDGain, _x.velocityTimeToTargetS))
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
      start = end
      end += length
      self.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.type = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.updateIntervalS, _x.cmdTimeoutS, _x.maxVelocity, _x.minPosition, _x.maxPosition, _x.initialPosition, _x.positionPGain, _x.positionDGain, _x.velocityTimeToTargetS,) = _struct_9f.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_9f = struct.Struct("<9f")
