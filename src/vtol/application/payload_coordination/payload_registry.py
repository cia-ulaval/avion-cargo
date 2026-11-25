from typing import Protocol

from shared.contracts.geometry import Heading
from shared.contracts.messages import DLZState, PayloadSnapshot
from vtol.domain.payload_peer import PayloadPeer


class PayloadRegistry(Protocol):
    """
    Maintains the plane application's latest view of all known payloads.
    """

    async def add(self, peer: PayloadPeer) -> None:
        """
        Register a payload peer and begin consuming its event stream.
        """
        ...

    async def remove(self, payload_id: str) -> None:
        """
        Remove a payload from the registry.
        """
        ...

    def get_dlz_state(self) -> DLZState:
        """
        Return the latest locally maintained DLZ snapshot.
        """
        ...

    def get_nearby_payloads(self) -> tuple[PayloadSnapshot, ...]:
        """
        Return payloads which are nearby and reachable.
        """
        ...

    def get_docked_payload(self) -> PayloadSnapshot | None:
        """
        Return the payload that is currently docked onto the vtol.
        """
        ...

    async def refresh_states(self) -> DLZState:
        """
        Request and aggregate fresh status reports from connected payloads.
        """
        ...

    async def broadcast_heading(self, heading: Heading, sequence: int) -> None:
        """
        Broadcast a heading update to registered payloads.
        """
        ...

    async def broadcast_dlz_state(self) -> None:
        """
        Broadcast the latest DLZ snapshot to registered payloads.
        """
        ...
