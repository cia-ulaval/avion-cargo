from __future__ import annotations

import asyncio
import inspect
import threading
from dataclasses import dataclass
from typing import Any

from d3kble import D3KNode, IncomingRequest
from loguru import logger


@dataclass
class _PeerRoute:
    message_handler: Any = None
    request_handler: Any = None
    token: object | None = None


class D3kbleRouteRegistration:
    """
    Handle used to remove one peer's handlers from a shared router.
    """
    def __init__(self, router: "D3kbleMessageRouter", peer_id: int, token: object):
        self._router = router
        self._peer_id = peer_id
        self._token = token
        self._closed = False

    def close(self) -> None:
        if not self._closed:
            self._router._unregister(self._peer_id, self._token)
            self._closed = True


class D3kbleMessageRouter:
    """
    Route incoming d3kble messages and requests by remote BLE peer ID.
    """
    def __init__(self, node: D3KNode):
        self._node = node
        self._routes: dict[int, _PeerRoute] = {}
        self._lock = threading.RLock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._started = False
        self._closed = False

    async def start(self) -> None:
        """
        Bind d3kble async handlers to the current application event loop.
        """
        loop = asyncio.get_running_loop()
        with self._lock:
            if self._closed:
                raise RuntimeError("a closed D3kbleMessageRouter cannot be restarted")
            
            if self._started:
                if self._loop is not loop:
                    raise RuntimeError("D3kbleMessageRouter is already bound to another event loop")
                
                return
            
            self._node.comms.bind_asyncio_loop(loop)
            self._node.comms.on_message(self._route_message)
            self._node.comms.on_request(self._route_request)
            self._loop = loop
            self._started = True

    def register_peer(
        self,
        peer_id: int,
        *,
        message_handler=None,
        request_handler=None,
    ) -> D3kbleRouteRegistration:
        
        if isinstance(peer_id, bool) or not isinstance(peer_id, int) or not 1 <= peer_id <= 65534:
            raise ValueError("peer_id must be an integer in 1..65534")
        
        if message_handler is None and request_handler is None:
            raise ValueError("at least one handler must be supplied")
        
        token = object()
        
        with self._lock:
            if self._closed:
                raise RuntimeError("router is closed")
            
            if not self._started:
                raise RuntimeError("start the router before registering peer routes")
            
            if peer_id in self._routes:
                raise ValueError(f"a route for BLE peer {peer_id} is already registered")
            
            self._routes[peer_id] = _PeerRoute(message_handler, request_handler, token)
        
        return D3kbleRouteRegistration(self, peer_id, token)

    def _unregister(self, peer_id: int, token: object) -> None:
        with self._lock:
            route = self._routes.get(peer_id)

            if route is not None and route.token is token:
                self._routes.pop(peer_id, None)

    def _route_message(self, message: Any) -> None:
        with self._lock:
            route = self._routes.get(int(message.peer_id))
            handler = route.message_handler if route is not None else None

        if handler is None:
            return

        try:
            handler(message)
        except Exception:
            logger.exception("d3kble message handler failed for peer %s", message.peer_id)

    async def _route_request(self, request: IncomingRequest) -> Any:
        with self._lock:
            route = self._routes.get(int(request.peer_id))
            handler = route.request_handler if route is not None else None

        if handler is None:
            request.fail(f"no request handler registered for BLE peer {request.peer_id}")
            return None
        
        result = handler(request)

        if inspect.isawaitable(result):
            return await result
        
        return result

    def close(self) -> None:
        """
        Disable all routes; the single callbacks remain until D3KNode closes.
        """
        with self._lock:
            self._closed = True
            self._routes.clear()
