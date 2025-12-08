# std_srvs/srv/Trigger.py
from typing import Any

class TriggerRequest:
    """Stub for std_srvs.srv.TriggerRequest"""
    __slots__ = ()  # no fields
    _type = "std_srvs/Trigger"
    _md5sum = "d8a4b0c8d1b6347d8a33d0f3c0e7f2e5"

    def __init__(self) -> None:
        ...

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}()"


class TriggerResponse:
    """Stub for std_srvs.srv.TriggerResponse"""
    __slots__ = ("success", "message")
    _type = "std_srvs/Trigger"
    _md5sum = "d8a4b0c8d1b6347d8a33d0f3c0e7f2e5"

    success: bool
    message: str

    def __init__(self, success: bool = False, message: str = "") -> None:
        self.success = success
        self.message = message

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(success={self.success}, message={self.message!r})"


class Trigger:
    """Stub for std_srvs.srv.Trigger service"""
    Request = TriggerRequest
    Response = TriggerResponse


class Request:
    pass