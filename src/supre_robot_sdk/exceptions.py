class SupreRobotSdkError(RuntimeError):
    """Base exception for the SDK."""


class ConfigurationError(SupreRobotSdkError):
    """Raised when config parsing or validation fails."""


class DependencyUnavailableError(SupreRobotSdkError):
    """Raised when a required native binding is not installed."""


class HardwareInitError(SupreRobotSdkError):
    """Raised when hardware initialization fails."""


class HardwareActivationError(SupreRobotSdkError):
    """Raised when hardware activation fails."""


class NotConnectedError(SupreRobotSdkError):
    """Raised when an operation requires an active connection."""

