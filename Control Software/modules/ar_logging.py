import logging

class CustomOutputHandler(logging.Handler):
    def __init__(self, output_func):
        super().__init__()
        self.output_func = output_func  # e.g., a function to send logs elsewhere

    def emit(self, record):
        try:
            log_entry = self.format(record)
            self.output_func(log_entry)
        except Exception:
            self.handleError(record)

