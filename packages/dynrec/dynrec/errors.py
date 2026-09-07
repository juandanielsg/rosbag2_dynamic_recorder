# Copyright 2026 juandanielsg
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

"""What can go wrong, as a family a caller can catch at whatever granularity it needs.

Everything raised by this library derives from :class:`DynrecError`, so a script that only wants
to log and carry on catches one thing. The distinctions below exist because they call for
different responses: a recorder that is not there yet is worth retrying, a call the recorder
refused is not.
"""


class DynrecError(Exception):
    """Base class for every failure this library raises."""


class RecorderNotFound(DynrecError):
    """No recorder on the graph, or none under the requested name."""


class AmbiguousRecorder(DynrecError):
    """Several recorders are running and none was named.

    Guessing would be worse than failing: the two recorders on a robot are usually recording
    different things, and picking the wrong one is silent.
    """

    def __init__(self, message, recorders=()):
        super().__init__(message)
        #: The recorder node names that were found, so a caller can choose without re-discovering.
        self.recorders = list(recorders)


class ServiceUnavailable(DynrecError):
    """The recorder is known but the service did not appear in time."""


class CallTimeout(DynrecError):
    """The service was reached but did not reply in time.

    Distinct from ServiceUnavailable on purpose: this one means the recorder is up and accepting
    calls but did not finish this one, which points at a busy or wedged recorder rather than at a
    name or a domain mismatch.
    """


class CallFailed(DynrecError):
    """The recorder refused the call.

    Carries the service's own `return_code` and `error_string`, because the recorder's message is
    almost always more specific than anything this library could say on its behalf.
    """

    def __init__(self, message, return_code=None, error_string=''):
        super().__init__(message)
        self.return_code = return_code
        self.error_string = error_string


class InvalidRequest(DynrecError, ValueError):
    """The request could not be built from what the caller passed.

    Also a ValueError, since that is what a caller checking its own arguments will reach for, and
    every instance of this really is a mistake in the calling code rather than a recorder fault.
    """
