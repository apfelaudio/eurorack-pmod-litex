#!/usr/bin/env python3

import lxbuildenv
import rtl.dsp

import pytest

pytest.main(["-x", "rtl/dsp.py", "-vv", "-s"])
