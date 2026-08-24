class _FakeBlob:
    def __init__(self, x, y, width, height, pixels):
        self._x = x
        self._y = y
        self._width = width
        self._height = height
        self._pixels = pixels

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._width

    def h(self):
        return self._height

    def pixels(self):
        return self._pixels

    def area(self):
        return self._width * self._height


class _FakeSerial:
    def __init__(self):
        self.sent = []

    def write(self, data):
        self.sent.append(bytes(data))


def _selftest():
    process_command_bytes(b"1\r\n")
    assert current_mode == 1
    process_command_bytes(b"2\r\n")
    assert current_mode == 2
    process_command_bytes(b"x\r\n")
    assert current_mode == 2

    valid = _FakeBlob(120, 80, 50, 50, 1800)
    smaller = _FakeBlob(150, 90, 30, 30, 900)
    non_square = _FakeBlob(150, 90, 100, 20, 1800)
    assert filter_blob(non_square) is None
    assert select_best_blob([smaller, valid]) is valid

    serial = _FakeSerial()
    set_mode(1, "test")
    for _ in range(3):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n"]
    for _ in range(3):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n"]

    for _ in range(5):
        update_detection(None, serial)
    for _ in range(3):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n", b"1\n"]

    set_mode(1, "new-task")
    for _ in range(3):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n", b"1\n", b"1\n"]
    print("red-blue ball selftest passed")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _selftest()
