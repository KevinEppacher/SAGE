int32 id                  # Unique ID for event or node tick
bool success              # True if operation succeeded, False if failed
string comment            # Free-form description (e.g. "Timeout in SeekoutGraphNodes")
string node_name          # Optional: which BT node or module emitted the message
builtin_interfaces/Time stamp  # When the event occurred