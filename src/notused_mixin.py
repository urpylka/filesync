#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

# Copyright 2018-2019 Artem Smirnov

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

class ParamsMixin(StateMixin):
    """
    Parameters saver/reader mixin, using ``name`` field as a slug
    """
    PARAMS_FILE = config.PARAMS_FILE

    def __init__(self):
        super(ParamsMixin, self).__init__()
        self.params = ParamsStorage(self.PARAMS_FILE)

    def get_state(self):
        return self.update_state(ParamsMixin, {'params': self.params.params_dict})
