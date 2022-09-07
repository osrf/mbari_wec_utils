# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import os

from ament_copyright.licenses import read_license_data


TEMPLATE_DIRECTORY = os.path.join(os.path.dirname(__file__), 'template')

mpl2 = read_license_data(TEMPLATE_DIRECTORY, 'Mozilla Public License, Version 2.0', 'mpl2')
