# drivers/kb_driver.py
# Keyboard driver (pynput) returning compact 1:1 rover commands.
#
# Bindings (exactly as requested):
#   W or UpArrow     -> "f"   (forward)
#   S or DownArrow   -> "b"   (backward)
#   LeftArrow        -> "l"   (pivot left)
#   RightArrow       -> "r"   (pivot right)
#   A                -> "pl"  (pivot-left via letter 'A')
#   D                -> "pr"  (pivot-right via letter 'D')
#   Q                -> "fpl" (forward + pivot left)
#   E                -> "fpr" (forward + pivot right)
#   Z                -> "bpl" (backward + pivot left)
#   X                -> "bpr" (backward + pivot right)
#   U                -> "fl"  (forward-left / crab left)
#   I                -> "fr"  (forward-right / crab right)
#   J                -> "bl"  (backward-left  / crab left backward)
#   K                -> "br"  (backward-right / crab right backward)
#
# Priority rule when multiple keys are held:
#   1) Diagonal/special combos (Q/E/Z/X/U/I/J/K)
#   2) A/D (pl/pr)
#   3) Left/Right arrows (l/r)
#   4) W/Up and S/Down (f/b)
# If nothing is pressed, returns None.

from pynput import keyboard


class KeyboardDriverPynput:
    """Global keyboard input reader that returns 1:1 rover command strings."""

    def __init__(self):
        # We store normalized tokens like: 'w','a','q','arrow_up','arrow_left',...
        self._pressed = set()
        self._listener = None

    # ------------- internal: tokenization helpers -----------------
    @staticmethod
    def _arrow_token(key):
        if key == keyboard.Key.up:
            return "arrow_up"
        if key == keyboard.Key.down:
            return "arrow_down"
        if key == keyboard.Key.left:
            return "arrow_left"
        if key == keyboard.Key.right:
            return "arrow_right"
        return None

    def _on_press(self, key):
        try:
            # character keys
            ch = key.char.lower()
            if len(ch) == 1 and "a" <= ch <= "z":
                self._pressed.add(ch)
        except AttributeError:
            # special keys (arrows)
            tok = self._arrow_token(key)
            if tok:
                self._pressed.add(tok)

    def _on_release(self, key):
        try:
            ch = key.char.lower()
            if len(ch) == 1 and "a" <= ch <= "z":
                self._pressed.discard(ch)
        except AttributeError:
            tok = self._arrow_token(key)
            if tok:
                self._pressed.discard(tok)

    # ------------- public API -------------------------------------
    def start(self):
        if self._listener:
            return
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

    def stop(self):
        if self._listener:
            self._listener.stop()
            self._listener = None

    def read_dir(self):
        """
        Returns one of:
          "f","b","l","r",
          "pl","pr",
          "fpl","fpr","bpl","bpr",
          "fl","fr","bl","br",
        or None if no relevant key is pressed.
        """
        p = self._pressed

        # 1) Highest priority: dedicated diagonals/special combos
        if "q" in p: return "fpl"
        if "e" in p: return "fpr"
        if "z" in p: return "bpl"
        if "x" in p: return "bpr"
        if "u" in p: return "fl"
        if "i" in p: return "fr"
        if "j" in p: return "bl"
        if "k" in p: return "br"

        # 2) A/D pivot via letters
        if "a" in p: return "pl"
        if "d" in p: return "pr"

        # 3) Left/Right arrows (pivot)
        if "arrow_left" in p: return "l"
        if "arrow_right" in p: return "r"

        # 4) Forward/backward (W or Up, S or Down)
        if "w" in p or "arrow_up" in p: return "f"
        if "s" in p or "arrow_down" in p: return "b"

        return None
