"""Release an SSH endpoint's resources if either protocol pipe disconnects."""
from contextlib import contextmanager
import os
import select
import threading


@contextmanager
def watch_connection(input_fd=0, output_fd=1):
    # Check hangups without consuming predictions. This also runs while the
    # motor thread is waiting for the initial pose or executing a chunk.
    poller = select.poll()
    lost = select.POLLHUP | select.POLLERR | select.POLLNVAL
    poller.register(input_fd, lost)
    poller.register(output_fd, lost)
    stopped = threading.Event()

    def watch():
        while not stopped.is_set():
            if poller.poll(100) and not stopped.is_set():
                # Do not unwind return_to_pose: its error cleanup releases
                # torque and can drop an unsupported arm. Kernel exit closes
                # this process's serial/camera descriptors and advisory lock.
                try:
                    os.write(2, b'ACT connection closed; releasing serial port without motor or torque commands.\n')
                finally:
                    os._exit(130)

    worker = threading.Thread(target=watch, daemon=True)
    worker.start()
    try:
        yield
    finally:
        stopped.set()
        worker.join()
