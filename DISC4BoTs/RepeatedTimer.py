from threading import Timer

class RepeatedTimer(object):
    """
		An object used to execute a function every certain amount of time
        thanks to: https://stackoverflow.com/questions/3393612/run-certain-code-every-n-seconds

		Attributes:
		interval(float): the function runs every [interval] seconds
        function(function): the function to run
        args & kwargs: function parameters
        is_running (bool): if True, the function is executed every [interval] seconds

		Methods:
		run() 
			Runs the function
		start()
			Starts running the function every certain amount of time
        stop()
            Stops the repeating run of the function
    """

    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False