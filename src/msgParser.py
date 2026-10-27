"""
msgParser.py - TORCS SCR message parser / action stringifier.

The TORCS server (scr_server) exchanges UDP datagrams formatted as a
sequence of parenthesised groups:

    (angle 0.123)(speedX 87.5)(track 200.0 180.0 ...)(...)

This module converts such a message into a plain dict of floats / lists
(the "state") and converts a control dict back into the wire format:

    (accel 1.0)(brake 0.0)(clutch 0.0)(gear 3)(steer -0.12)(focus 0)(meta 0)

It is intentionally dependency-free (standard library only).
"""


class msgParser(object):
    """Parse TORCS sensor messages and stringify control actions."""

    def __init__(self):
        self.state = {}

    # ------------------------------------------------------------------
    # Incoming: server state message -> dict
    # ------------------------------------------------------------------
    def parse(self, msg):
        """Parse one server message into self.state and return it."""
        state = {}
        # Split on '(' -> ['', 'angle 0.1)', 'speedX 12)', ...]
        for chunk in msg.split('('):
            chunk = chunk.strip().rstrip(')')
            if not chunk:
                continue
            parts = chunk.split()
            if len(parts) < 2:
                continue
            key = parts[0]
            values = []
            for token in parts[1:]:
                try:
                    values.append(float(token))
                except ValueError:
                    values.append(token)
            state[key] = values[0] if len(values) == 1 else values
        self.state = state
        return state

    # ------------------------------------------------------------------
    # Outgoing: control dict -> wire string
    # ------------------------------------------------------------------
    def stringify(self, actions):
        """Build the control string from a dict of control values."""
        out = []
        for key in ('accel', 'brake', 'clutch', 'gear', 'steer', 'focus', 'meta'):
            if key in actions:
                value = actions[key]
                if key in ('gear', 'meta'):
                    value = int(value)
                out.append('(%s %s)' % (key, value))
        return ''.join(out)
