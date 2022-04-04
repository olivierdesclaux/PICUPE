import multiprocessing
import traceback


class Process(multiprocessing.Process):
    """
    Class which returns child Exceptions to Parent.
    Taken from this stackoverflow question:
    https://stackoverflow.com/a/58060759/14583450
    """

    def __init__(self, name, logger, system, *args, **kwargs):
        multiprocessing.Process.__init__(self, *args, **kwargs)
        self._parent_conn, self._child_conn = multiprocessing.Pipe()
        self._exception = None
        self.name = name
        self.logger = logger
        self.system = system

    def run(self):
        try:
            multiprocessing.Process.run(self)
            self._child_conn.send(None)
        except Exception as e:
            tb = traceback.format_exc()
            self.logger.log(tb)
            self._child_conn.send((e, tb))
            # myLogger.log()
            # raise e  # You can still rise this exception if you need to

    def terminate(self):
        self.system.close()
        multiprocessing.Process.terminate(self)

    @property
    def exception(self):
        if self._parent_conn.poll():
            self._exception = self._parent_conn.recv()
        return self._exception


def monitorProcesses(procs):
    """
    Function that will be threaded in the main process. Verifies that all child processes are running correctly. If
    one of the child processes fails, will terminate all child processes. Terminates in the end after all child
    processes have been terminated.

    Parameters
    ----------
    procs: list, list of processes

    Returns
    -------
    None
    """
    if any([proc.is_alive() for proc in procs]):
        for proc in procs:
            if proc.exception:
                error, procTraceback = proc.exception
                raise ChildProcessError(procTraceback)
