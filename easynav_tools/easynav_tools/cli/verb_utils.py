# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import re

_TAGS = {
    'black': '30', 'red': '31', 'green': '32', 'yellow': '33',
    'blue': '34', 'magenta': '35', 'cyan': '36', 'white': '37',
    'bold': '1', 'dim': '2', 'underline': '4'
}

_OPEN_TAG_RE = re.compile(r'\[([a-zA-Z]+)\]')
_CLOSE_TAG_RE = re.compile(r'\[/([a-zA-Z]+)\]')


def bbcode_to_ansi(s: str, enable_color: bool) -> str:
    if not enable_color:
        s = _OPEN_TAG_RE.sub('', s)
        s = _CLOSE_TAG_RE.sub('', s)
        return s

    for tag, code in _TAGS.items():
        s = re.sub(
            rf'\[{tag}\](.*?)\[/\s*{tag}\]',
            lambda m: f'\033[{code}m{m.group(1)}\033[0m',
            s,
            flags=re.DOTALL | re.IGNORECASE
        )
    s = _OPEN_TAG_RE.sub('', s)
    s = _CLOSE_TAG_RE.sub('', s)
    return s
