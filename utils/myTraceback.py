import multiprocessing
import traceback
import myLogger
from time import sleep
import threading
import msvcrt

class Process(multiprocessing.Process):
    """
    Class which returns child Exceptions to Parent.
    Taken from this stackoverflow question:
    https://stackoverflow.com/a/58060759/14583450
    """

    def __init__(self, name, loggerQueue, *args, **kwargs):
        multiprocessing.Process.__init__(self, *args, **kwargs)
        self._parent_conn, self._child_conn = multiprocessing.Pipe()
        self._exception = None
        self.name = name
        self.loggerQueue = loggerQueue

    def run(self):
        try:
            multiprocessing.Process.run(self)
            self._child_conn.send(None)
        except Exception as e:
            tb = traceback.format_exc()
            myLogger.log(self.loggerQueue, tb)
            self._child_conn.send((e, tb))
            # myLogger.log()
            # raise e  # You can still rise this exception if you need to

    @property
    def exception(self):
        if self._parent_conn.poll():
            self._exception = self._parent_conn.recv()
        return self._exception


class Task_1:
    def do_something(self, queue):
        queue.put(dict(users=2))
        sleep(5)
        try:
            x = 1/0
        except Exception as e:
            raise e


class Task_2:
    def do_something(self, queue):
        queue.put(dict(users=5))
        sleep(8)
        x = 1/0

def monitorProcesses(procs, loggerQueue):
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
    # while any([proc.is_alive() for proc in procs]):
    if any([proc.is_alive() for proc in procs]):
        for proc in procs:
            # myLogger.log(loggerQueue, proc.name)
            if proc.exception:
                error, procTraceback = proc.exception
                for allOtherProc in procs:
                    allOtherProc.terminate()
                # myLogger.log(loggerQueue, procTraceback)
                raise ChildProcessError(procTraceback)

    # print("Finished monitoring child processes.")


def main():
    customLog = myLogger.initLogger("./", "traceback.log")
    loggerQueue = multiprocessing.Queue()
    listener = threading.Thread(target=myLogger.logWriter, args=(customLog, loggerQueue), daemon=True)
    listener.start()

    try:
        task_1 = Task_1()
        task_2 = Task_2()

        # Example of multiprocessing which is used:
        # https://eli.thegreenplace.net/2012/01/16/python-parallelizing-cpu-bound-tasks-with-multiprocessing/
        task_1_queue = multiprocessing.Queue()
        task_2_queue = multiprocessing.Queue()

        task_1_process = Process("task 1", loggerQueue, target=task_1.do_something, kwargs=dict(queue=task_1_queue))
        # task_1_process.name = "task 1"
        # task_1_process.loggerQueue = loggerQueue
        task_2_process = Process("task 2", loggerQueue, target=task_2.do_something, kwargs=dict(queue=task_2_queue))
        # task_2_process.name = "task 2"
        # task_2_process.loggerQueue = loggerQueue

        procs = [task_1_process, task_2_process]
        for proc in procs:
            proc.start()

        while True:
            monitorProcesses(procs, loggerQueue)
            if msvcrt.kbhit():
                if msvcrt.getwche() == '\r':
                    myLogger.log(loggerQueue, "\nStopping recording... \n")
                    break

        task_1_process.join()
        task_2_process.join()

    except Exception:
        # Here usually I send email notification with error.
        myLogger.log(loggerQueue, traceback.format_exc())
        sleep(0.5)
        loggerQueue.put_nowait(None)


if __name__ == "__main__":
    main()