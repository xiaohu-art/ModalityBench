from typing import Any, Protocol


class Policy(Protocol):
    """Produces actions without modifying simulator state."""

    def act(self, observation: Any) -> Any | None:
        """None means no new command."""
        ...
