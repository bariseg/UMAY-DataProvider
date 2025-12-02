from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class FlightData(_message.Message):
    __slots__ = ("latitude", "longitude", "altitude", "speed", "heading", "battery", "timestamp")
    LATITUDE_FIELD_NUMBER: _ClassVar[int]
    LONGITUDE_FIELD_NUMBER: _ClassVar[int]
    ALTITUDE_FIELD_NUMBER: _ClassVar[int]
    SPEED_FIELD_NUMBER: _ClassVar[int]
    HEADING_FIELD_NUMBER: _ClassVar[int]
    BATTERY_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    latitude: float
    longitude: float
    altitude: float
    speed: float
    heading: float
    battery: float
    timestamp: int
    def __init__(self, latitude: _Optional[float] = ..., longitude: _Optional[float] = ..., altitude: _Optional[float] = ..., speed: _Optional[float] = ..., heading: _Optional[float] = ..., battery: _Optional[float] = ..., timestamp: _Optional[int] = ...) -> None: ...
