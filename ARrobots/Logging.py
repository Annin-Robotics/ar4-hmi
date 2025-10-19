import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.NOTSET) # Inherit level from Parent

class CustomOutputHandler(logging.Handler):
    def __init__(self, output_func):
        super().__init__()
        self.output_func = output_func  # e.g., a function to send logs elsewhere

    def emit(self, record):
        try:
            log_entry = self.format(record)
            self.output_func(log_entry)
        except Exception as e:
            print(f"Error handling log event: {e}")
            #self.handleError(record)

class ModuleFilter(logging.Filter):
    def __init__(self, *blocked_prefixes):
        super().__init__()
        self.blocked_prefixes = tuple(blocked_prefixes)
    def filter(self, record):
        return not record.name.startswith(self.blocked_prefixes)
    
def dump_logger_info(name):
    import logging
    lg = logging.getLogger(name)
    print(f"Logger: {name}")
    print(f"  level: {logging.getLevelName(lg.level)} (raw)")
    print(f"  effective: {logging.getLevelName(lg.getEffectiveLevel())}")
    print(f"  propagate: {lg.propagate}")
    print(f"  handlers: {[type(h).__name__+':' + logging.getLevelName(h.level) for h in lg.handlers]}")
    if lg.parent:
        print(f"  parent: {lg.parent.name}")

