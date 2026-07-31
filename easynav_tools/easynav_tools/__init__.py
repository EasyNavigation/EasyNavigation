import os
import sys
import unittest

_vendor_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), 'vendor'))
if os.path.isdir(_vendor_dir) and _vendor_dir not in sys.path:
    sys.path.insert(0, _vendor_dir)


def load_tests(loader, tests, pattern):
    return unittest.TestSuite()
