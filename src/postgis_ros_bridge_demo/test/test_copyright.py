# Copyright 2023 AIT - Austrian Institute of Technology GmbH
# Copyright 2015 Open Source Robotics Foundation, Inc.
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

from pathlib import Path

from ament_copyright.main import main
import pytest


# Remove the `skip` decorator once the source file(s) have a copyright header
# @pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Test source code for license compliance."""
    pkg_prefix = str(Path(__file__).parents[1])
    rci = main(argv=[pkg_prefix, 'test'])
    assert rci == 0, 'Found errors'
