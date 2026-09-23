"""Length-prefixed JSON + optional binary payload for the local split processes."""

from __future__ import annotations

import json
import socket
import struct
from typing import Any, Dict, Tuple


MAX_JSON_BYTES = 1024 * 1024
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("对端已断开连接")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_packet(sock: socket.socket, message: Dict[str, Any], payload: bytes = b"") -> None:
    envelope = dict(message)
    envelope["payload_size"] = len(payload)
    encoded = json.dumps(envelope, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_JSON_BYTES:
        raise ValueError("JSON 消息过大")
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("二进制负载过大")
    sock.sendall(struct.pack("!I", len(encoded)))
    sock.sendall(encoded)
    if payload:
        sock.sendall(payload)


def recv_packet(sock: socket.socket) -> Tuple[Dict[str, Any], bytes]:
    json_size = struct.unpack("!I", _recv_exact(sock, 4))[0]
    if json_size > MAX_JSON_BYTES:
        raise ValueError("收到的 JSON 消息过大")
    message = json.loads(_recv_exact(sock, json_size).decode("utf-8"))
    payload_size = int(message.pop("payload_size", 0))
    if payload_size < 0 or payload_size > MAX_PAYLOAD_BYTES:
        raise ValueError("收到的二进制负载大小非法")
    return message, _recv_exact(sock, payload_size) if payload_size else b""
