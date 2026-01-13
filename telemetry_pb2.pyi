from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class TargetData(_message.Message):
    __slots__ = ("distanceHorizontalMax", "distanceVerticalMax", "targets")
    DISTANCEHORIZONTALMAX_FIELD_NUMBER: _ClassVar[int]
    DISTANCEVERTICALMAX_FIELD_NUMBER: _ClassVar[int]
    TARGETS_FIELD_NUMBER: _ClassVar[int]
    distanceHorizontalMax: float
    distanceVerticalMax: float
    targets: _containers.RepeatedCompositeFieldContainer[Target]
    def __init__(self, distanceHorizontalMax: _Optional[float] = ..., distanceVerticalMax: _Optional[float] = ..., targets: _Optional[_Iterable[_Union[Target, _Mapping]]] = ...) -> None: ...

class Target(_message.Message):
    __slots__ = ("id", "distanceHorizontal", "distanceVertical")
    ID_FIELD_NUMBER: _ClassVar[int]
    DISTANCEHORIZONTAL_FIELD_NUMBER: _ClassVar[int]
    DISTANCEVERTICAL_FIELD_NUMBER: _ClassVar[int]
    id: int
    distanceHorizontal: float
    distanceVertical: float
    def __init__(self, id: _Optional[int] = ..., distanceHorizontal: _Optional[float] = ..., distanceVertical: _Optional[float] = ...) -> None: ...

class FlightData(_message.Message):
    __slots__ = ("latitude", "longitude", "altitude", "speed", "heading", "battery", "timestamp", "roll", "pitch", "targetData", "vertical_speed", "current", "voltage", "gps_satellites", "gps_hdop", "gps_fix_type", "flight_mode", "vibration_x", "vibration_y", "vibration_z")
    LATITUDE_FIELD_NUMBER: _ClassVar[int]
    LONGITUDE_FIELD_NUMBER: _ClassVar[int]
    ALTITUDE_FIELD_NUMBER: _ClassVar[int]
    SPEED_FIELD_NUMBER: _ClassVar[int]
    HEADING_FIELD_NUMBER: _ClassVar[int]
    BATTERY_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    ROLL_FIELD_NUMBER: _ClassVar[int]
    PITCH_FIELD_NUMBER: _ClassVar[int]
    TARGETDATA_FIELD_NUMBER: _ClassVar[int]
    VERTICAL_SPEED_FIELD_NUMBER: _ClassVar[int]
    CURRENT_FIELD_NUMBER: _ClassVar[int]
    VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    GPS_SATELLITES_FIELD_NUMBER: _ClassVar[int]
    GPS_HDOP_FIELD_NUMBER: _ClassVar[int]
    GPS_FIX_TYPE_FIELD_NUMBER: _ClassVar[int]
    FLIGHT_MODE_FIELD_NUMBER: _ClassVar[int]
    VIBRATION_X_FIELD_NUMBER: _ClassVar[int]
    VIBRATION_Y_FIELD_NUMBER: _ClassVar[int]
    VIBRATION_Z_FIELD_NUMBER: _ClassVar[int]
    latitude: float
    longitude: float
    altitude: float
    speed: float
    heading: float
    battery: float
    timestamp: int
    roll: float
    pitch: float
    targetData: TargetData
    vertical_speed: float
    current: float
    voltage: float
    gps_satellites: int
    gps_hdop: float
    gps_fix_type: int
    flight_mode: str
    vibration_x: float
    vibration_y: float
    vibration_z: float
    def __init__(self, latitude: _Optional[float] = ..., longitude: _Optional[float] = ..., altitude: _Optional[float] = ..., speed: _Optional[float] = ..., heading: _Optional[float] = ..., battery: _Optional[float] = ..., timestamp: _Optional[int] = ..., roll: _Optional[float] = ..., pitch: _Optional[float] = ..., targetData: _Optional[_Union[TargetData, _Mapping]] = ..., vertical_speed: _Optional[float] = ..., current: _Optional[float] = ..., voltage: _Optional[float] = ..., gps_satellites: _Optional[int] = ..., gps_hdop: _Optional[float] = ..., gps_fix_type: _Optional[int] = ..., flight_mode: _Optional[str] = ..., vibration_x: _Optional[float] = ..., vibration_y: _Optional[float] = ..., vibration_z: _Optional[float] = ...) -> None: ...
